import tempfile
import time
import unittest
import uuid
from pathlib import Path
from fastapi.testclient import TestClient
from app import create_app
from core.settings import load_config


class ApiTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.app=create_app(mock=True,journal_path=Path(self.tmp.name)/'execution.json')
        self.client=TestClient(self.app,base_url='http://localhost')
        self.client.__enter__()

    def tearDown(self):
        self.client.__exit__(None,None,None)
        self.tmp.cleanup()

    def claim(self):
        return self.client.post('/api/control/claim',json={})

    def post(self,path,**extra):
        s=self.client.get('/api/state').json()
        return self.client.post(path,json={'revision':s['revision'],'request_id':str(uuid.uuid4()),**extra})

    def test_startup_sends_nothing_and_assets(self):
        self.assertEqual(self.app.state.cycle.robot.client.commands,[])
        self.assertEqual(self.client.get('/').status_code,200)
        self.assertEqual(self.client.get('/static/app.js').status_code,200)
        self.assertEqual(self.client.get('/api/preview.jpg').status_code,503)
        self.assertEqual(self.client.get('/api/laser/status').json()['laser_in1'],'UNKNOWN')

    def test_control_requires_lease_and_single_hmi(self):
        self.assertEqual(self.post('/api/system/connect').status_code,423)
        self.assertEqual(self.claim().status_code,200)
        second=TestClient(self.app,base_url='http://localhost')
        self.assertEqual(second.post('/api/control/claim',json={}).status_code,423)
        self.assertEqual(second.get('/api/state').json()['control'],False)

    def test_cross_origin_and_invalid_host_rejected(self):
        self.assertEqual(self.client.post('/api/control/claim',json={},headers={'Origin':'http://evil.example'}).status_code,403)
        self.assertEqual(self.client.get('/api/state',headers={'Host':'evil.example'}).status_code,400)
        self.assertEqual(self.client.post('/api/control/claim',content='{}').status_code,415)

    def test_duplicate_and_stale_mutation(self):
        self.claim()
        s=self.client.get('/api/state').json()
        body={'revision':s['revision'],'request_id':'request-config-01','settings':{'rz':0}}
        self.assertEqual(self.client.post('/api/config',json=body).status_code,200)
        self.assertEqual(self.client.post('/api/config',json=body).status_code,409)
        body['request_id']='request-config-02'
        self.assertEqual(self.client.post('/api/config',json=body).status_code,409)

    def test_connect_preview_and_config(self):
        self.claim()
        self.assertEqual(self.post('/api/system/connect').status_code,200)
        time.sleep(.3)
        preview=self.client.get('/api/preview.jpg')
        self.assertEqual(preview.status_code,200)
        self.assertTrue(preview.content.startswith(b'\xff\xd8'))
        self.assertEqual(self.post('/api/config',settings={'z_safe':505}).status_code,200)
        self.assertEqual(self.post('/api/config',settings={'z_safe':'nan'}).status_code,422)
        self.assertEqual(self.post('/api/vacuum/3',on=True).status_code,422)

    def test_operating_settings_save_drop_geometry_without_reset_or_motion(self):
        from workflow.drop import StackStore
        import json
        self.claim()
        c=self.app.state.cycle
        c.stack.confirm('previous-board',picked_id=3)
        response=self.post('/api/config',settings={'z_safe':505},
                           drop_settings={'first_drop_z_mm':340.5,'board_thickness_mm':15.25})
        self.assertEqual(response.status_code,200,response.text)
        d=response.json()['drop_height']
        self.assertEqual((d['first_drop_z_mm'],d['board_thickness_mm'],d['stacked_count'],d['next_drop_z_mm']),
                         (340.5,15.25,1,355.75))
        self.assertEqual(d['last_picked_id'],3)
        saved=StackStore(c.stack.path,c.cfg['drop']).snapshot()
        self.assertEqual(saved,{key:d[key] for key in saved})
        self.assertEqual(json.loads((c.journal.path.parent/'operator_settings.json').read_text())['z_safe'],505)
        self.assertEqual(c.robot.client.commands,[])

    def test_operating_settings_reject_invalid_drop_before_changing_any_values(self):
        self.claim()
        c=self.app.state.cycle
        original=c.stack.snapshot()
        original_settings=dict(c.cfg['settings'])
        for values in ({'first_drop_z_mm':None},{'first_drop_z_mm':True},
                       {'first_drop_z_mm':601},{'first_drop_z_mm':-501},
                       {'first_drop_z_mm':330.1234},{'board_thickness_mm':0},
                       {'board_thickness_mm':1001},{'other':12}):
            with self.subTest(values=values):
                response=self.post('/api/config',settings={'z_safe':505},drop_settings=values)
                self.assertEqual(response.status_code,422,response.text)
                self.assertEqual(c.stack.snapshot(),original)
                self.assertEqual(c.cfg['settings'],original_settings)
        c.journal.update(state='HOLD')
        self.assertEqual(self.post('/api/config',settings={},drop_settings={'first_drop_z_mm':340}).status_code,409)
        self.assertEqual(c.stack.snapshot(),original)
        self.assertEqual(c.robot.client.commands,[])

    def test_lan_needs_token(self):
        cfg=load_config()
        cfg['web']['host']='0.0.0.0'
        with self.assertRaisesRegex(RuntimeError,'TOKEN'):
            create_app(config=cfg,journal_path=Path(self.tmp.name)/'second.json')

    def test_drop_config_and_reset_do_not_start_jobs_or_motion(self):
        self.assertEqual(self.post('/api/drop/settings',board_thickness_mm=15).status_code,423)
        self.claim()
        self.assertEqual(self.post('/api/drop/settings',board_thickness_mm=15).status_code,200)
        self.assertEqual(self.post('/api/drop/settings',board_thickness_mm=0).status_code,422)
        c=self.app.state.cycle
        c.stack.confirm('previous-drop')
        self.assertEqual(self.post('/api/drop/reset',confirmed=False).status_code,422)
        self.assertEqual(self.post('/api/drop/reset',confirmed=True).status_code,200)
        stack=self.client.get('/api/state').json()['drop_height']
        self.assertEqual((stack['stacked_count'],stack['next_drop_z_mm'],stack['board_thickness_mm']),(0,330,15))
        self.assertEqual(c.robot.client.commands,[])

    def test_drop_confirmation_requires_job_receipt_and_counts_once(self):
        self.claim()
        c=self.app.state.cycle
        c.robot.connect()
        c.journal.update(cycle_id='drop-api-001')
        self.assertEqual(self.post('/api/drop/confirm',cycle_id='drop-api-001',confirmed=True).status_code,409)
        c.journal.update(state='AWAIT_DROP_CONFIRMATION',drop_release=dict(cycle_id='drop-api-001',job_completed=True))
        self.assertEqual(self.post('/api/drop/confirm',cycle_id='other-cycle',confirmed=True).status_code,409)
        self.assertEqual(self.post('/api/drop/confirm',cycle_id='drop-api-001',confirmed=False).status_code,422)
        self.assertEqual(self.post('/api/drop/confirm',cycle_id='drop-api-001',confirmed=True).status_code,200)
        self.assertEqual(self.post('/api/drop/confirm',cycle_id='drop-api-001',confirmed=True).status_code,200)
        self.assertEqual(c.stack.snapshot()['stacked_count'],1)
        self.assertFalse(any(n in ('START','MOVL') for n,d in c.robot.client.commands))

    def test_edit_stack_count_recomputes_height_and_preserves_history(self):
        self.assertEqual(self.post('/api/drop/count',stacked_count=7).status_code,423)
        self.claim()
        c=self.app.state.cycle
        c.stack.confirm('counted-before-edit',picked_id=2)
        self.assertEqual(self.post('/api/drop/count',stacked_count=7).status_code,200)
        d=c.stack.snapshot()
        self.assertEqual((d['stacked_count'],d['next_drop_z_mm'],d['last_picked_id']),(7,414,2))
        from workflow.drop import StackStore
        self.assertEqual(StackStore(c.stack.path,c.cfg['drop']).snapshot()['stacked_count'],7)
        self.assertFalse(c.stack.confirm('counted-before-edit'))
        self.assertEqual(c.stack.snapshot()['stacked_count'],7)
        self.assertEqual(self.post('/api/drop/count',stacked_count=2).status_code,200)
        self.assertEqual(c.stack.snapshot()['next_drop_z_mm'],354)
        self.assertEqual(c.robot.client.commands,[])

    def test_edit_stack_count_rejects_bad_values_and_active_robot(self):
        self.claim()
        for value in (-1,1.5,True,None,'7',9007199254740992):
            with self.subTest(value=value):
                self.assertEqual(self.post('/api/drop/count',stacked_count=value).status_code,422)
        c=self.app.state.cycle
        c.journal.update(state='WAIT_DROP_POSITION')
        self.assertEqual(self.post('/api/drop/count',stacked_count=7).status_code,409)
        c.journal.update(state='IDLE')
        c.robot.connect()
        c.robot.client.running=True
        self.assertEqual(self.post('/api/drop/count',stacked_count=7).status_code,409)
        self.assertEqual(c.stack.snapshot()['stacked_count'],0)
        self.assertFalse(any(n in ('MOVL','START','IOWRITE','HOLD') for n,d in c.robot.client.commands))

    def test_hold_release_requires_lease_confirmation_and_deduplicates(self):
        self.assertEqual(self.post('/api/cycle/release-hold',confirmed=True).status_code,423)
        self.claim()
        c=self.app.state.cycle
        c.robot.connect()
        c.robot.client.holding=True
        c.refresh()
        self.assertEqual(self.post('/api/cycle/release-hold').status_code,422)
        s=self.client.get('/api/state').json()
        body=dict(revision=s['revision'],request_id='release-hold-unique-01',confirmed=True)
        self.assertEqual(self.client.post('/api/cycle/release-hold',json=body).status_code,200)
        self.assertEqual(self.client.post('/api/cycle/release-hold',json=body).status_code,409)
        self.assertEqual(sum(n=='HOLD' and d=='0' for n,d in c.robot.client.commands),1)

    def test_hand_endpoints_dispatch_four_jobs_without_iowrite(self):
        self.claim()
        c=self.app.state.cycle
        c.robot.connect()
        c.robot.vacuum_config=load_config()['vacuum']
        c.cfg['motion']['poll_seconds']=.02
        c.refresh()
        for channel,on,name in [(1,True,'H1ON'),(1,False,'H1OFF'),(2,True,'H2ON'),(2,False,'H2OFF')]:
            response=self.post(f'/api/vacuum/{channel}',on=on)
            self.assertEqual(response.status_code,200,response.text)
            c.thread.join(timeout=2)
            self.assertFalse(c.busy)
            state=self.client.get('/api/state').json()
            self.assertEqual(state['manual_job'],dict(name=name,status='complete'))
            self.assertEqual(state['vacuum_method'],'job')
            self.assertEqual(state['state'],'IDLE',state['error'])
        commands=c.robot.client.commands
        self.assertEqual([d for n,d in commands if n=='START'],['H1ON','H1OFF','H2ON','H2OFF'])
        self.assertFalse(any(n in ('IOWRITE','MOVL') for n,_ in commands))

    def test_sensor_off_on_reaches_hmi_api_without_verified_flag(self):
        self.claim()
        self.assertEqual(self.post('/api/system/connect').status_code,200)
        c=self.app.state.cycle
        c.robot.laser_config['verified']=False
        for on in (False,True,False):
            c.robot.client.laser=on
            c.refresh()
            status=self.client.get('/api/laser/status').json()
            self.assertEqual(status['laser_in1'],'ON' if on else 'OFF')
            self.assertEqual(status['raw_byte'],int(on))
            self.assertFalse(status['mapping_verified'])
