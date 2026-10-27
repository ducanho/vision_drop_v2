import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from core.settings import load_config
from workflow.cycle import Cycle
from workflow.state_machine import Journal
from workflow.safety_checks import real_gate, ready


class CycleTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.cfg = load_config()
        self.cfg['workflow']['repeat_until_empty']=False
        self.cfg['drop'].pop('position_variable',None)
        self.cfg['drop']['first_drop_z_mm']=330
        # Retain regression coverage for the older fully monitored sequence.
        self.cfg['workflow']['end_after_moveunti_start']=False
        self.cfg['workflow'].pop('z_completion',None)
        # Shorter dwell only for fault-path tests. Production dwell tested separately.
        self.cfg['vision'].update(stable_frames=3,stable_seconds=.15)
        self.cfg['motion'].update(timeout=.8,job_timeout=.9,poll_seconds=.02,target_timeout=2)
        self.c = Cycle(self.cfg,True,Path(self.tmp.name)/'execution.json')
        self.c.connect()
        self.wait_for(lambda:self.c.vision.status()['connected'])

    def tearDown(self):
        self.c.close()
        self.tmp.cleanup()

    def wait_for(self,condition,timeout=5):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if condition():
                return
            time.sleep(.01)
        self.fail('Timeout waiting for condition')

    def start(self,stage='full_cycle',request='request-0001'):
        self.c.start(request,self.c.journal.state['revision'],stage)

    def finish(self):
        self.wait_for(lambda:not self.c.busy,timeout=8)
        if self.c.journal.state['state']=='AWAIT_DROP_CONFIRMATION':
            self.c.confirm_drop(self.c.journal.state['cycle_id'],confirmed=True)
        return self.c.journal.snapshot()

    def test_complete_one_target_one_movl_one_job_vacuum_kept(self):
        self.start()
        result=self.finish()
        self.assertEqual(result['state'],'COMPLETE',result['error'])
        self.assertEqual(result['target']['id'],1)
        commands=self.c.robot.client.commands
        self.assertEqual(sum(n=='MOVL' for n,_ in commands),2)
        self.assertEqual([d for n,d in commands if n=='START'],['MOVEUNTI','H2ON','DROP','H2OFF'])
        self.assertEqual(result['vacuum_job'],dict(name='H2ON',status='complete'))
        self.assertFalse(any(n=='IOWRITE' for n,_ in commands))
        self.assertEqual(self.c.robot.client.pose[2],530)
        self.assertIsNone(result['pending'])
        self.assertEqual([h['state'] for h in result['history']][-1],'COMPLETE')

    def test_production_ten_frames_two_seconds(self):
        self.c.cfg['vision'].update(stable_frames=10,stable_seconds=2)
        self.c.cfg['motion']['target_timeout']=4
        start=time.monotonic()
        self.start()
        self.assertEqual(self.finish()['state'],'COMPLETE')
        self.assertGreaterEqual(time.monotonic()-start,2)

    def test_v1_single_xyz_move_then_moveunti_then_h2on_at_stop_z(self):
        self.c.robot.client.pose[2]=600
        original=self.c.robot.start_hand_job
        def check_stopped(channel,on,config=None):
            if not on:
                return original(channel,on,config)
            self.assertEqual((channel,on),(2,True))
            self.assertFalse(self.c.robot.status()['running'])
            self.assertTrue(self.c.robot.laser())
            self.assertEqual(self.c.robot.pose()[0]['z'],470)
            return original(channel,on,config)
        with patch.object(self.c.robot,'start_hand_job',check_stopped):
            self.start()
            result=self.finish()
        self.assertEqual(result['state'],'COMPLETE',result['error'])
        commands=[(n,d) for n,d in self.c.robot.client.commands if n in ('LOADV','MOVL','START','IOWRITE','HOLD')]
        self.assertEqual([n for n,d in commands],['LOADV','START','START','START','MOVL','START','MOVL'])
        self.assertEqual([d for n,d in commands if n=='START'],['MOVEUNTI','H2ON','DROP','H2OFF'])
        fields=commands[0][1].split(',')
        self.assertEqual(list(map(float,fields[4:7])),[result['target']['pose'][a] for a in ('x','y','z')])
        self.assertEqual(float(fields[6]),self.c.cfg['settings']['z_safe'])
        self.assertEqual(self.c.robot.pose()[0]['z'],530)
        self.assertNotIn('MOVE_TO_SAFE',[h['state'] for h in result['history']])

    def test_v1_lost_h2on_reply_does_not_repeat_or_complete(self):
        original=self.c.robot.client.execute
        def lost(name,data=''):
            result=original(name,data)
            if name=='START' and data=='H2ON':
                from robot.protocol import OutcomeUnknown
                raise OutcomeUnknown('H2ON response lost')
            return result
        with patch.object(self.c.robot.client,'execute',lost):
            self.start()
            result=self.finish()
        self.assertEqual(result['state'],'FAULT')
        self.assertEqual(result['vacuum_job']['status'],'unknown')
        self.assertTrue(result['outcome_unknown'])
        self.assertEqual(result['pending']['payload'],'H2ON')
        self.assertEqual(sum(n=='START' and d=='H2ON' for n,d in self.c.robot.client.commands),1)

    def test_moveunti_ack_does_not_prove_p002_arrival(self):
        self.c.robot.client.scenario='move_timeout'
        self.start()
        result=self.finish()
        self.assertEqual(result['state'],'FAULT')
        self.assertIn('timeout',result['error'])
        self.assertFalse(any(n=='START' and d=='H2ON' for n,d in self.c.robot.client.commands))

    def test_job_missing(self):
        self.c.robot.client.available_job=False
        self.start()
        self.assertIn('không tồn tại',self.finish()['error'])
        self.assertFalse(any(n=='START' for n,_ in self.c.robot.client.commands))
        self.assertFalse(any(n=='HOLD' for n,_ in self.c.robot.client.commands))
        self.assertFalse(self.c.journal.state['outcome_unknown'])

    def test_unverified_real_job_blocked_before_write(self):
        self.c.mock=False
        with self.assertRaises(RuntimeError):
            self.start()
        self.assertFalse(any(n in ('MOVL','START','IOWRITE') for n,_ in self.c.robot.client.commands))

    def test_laser_never_triggers(self):
        self.c.robot.client.scenario='laser_timeout'
        self.start()
        result=self.finish()
        self.assertEqual(result['state'],'FAULT')
        self.assertIn('timeout',result['error'])
        self.assertEqual(self.c.robot.vacuum,[None,None])

    def test_laser_on_before_z(self):
        self.c.robot.client.scenario='early_laser'
        with self.assertRaisesRegex(RuntimeError,'Laser'):
            self.start()
        self.assertFalse(any(n in ('MOVL','START') for n,_ in self.c.robot.client.commands))

    def test_laser_appears_after_xy_before_job(self):
        original=self.c._job
        def early():
            self.c.robot.client.laser=True
            original()
        with patch.object(self.c,'_job',early):
            self.start()
            self.assertIn('trước Z',self.finish()['error'])
        self.assertFalse(any(n=='START' for n,_ in self.c.robot.client.commands))

    def test_unproven_job_stop_blocks_vacuum(self):
        self.c.robot.client.scenario='unproven_stop'
        self.start()
        self.assertIn('Chưa chứng minh',self.finish()['error'])
        self.assertFalse(any(n=='IOWRITE' or (n=='START' and d=='H2ON') for n,d in self.c.robot.client.commands))

    def test_hold_cancels_and_never_resumes(self):
        self.start()
        self.c.hold()
        self.assertEqual(self.finish()['state'],'HOLD')
        with self.assertRaises(RuntimeError):
            self.start(request='request-new2')
        self.assertFalse(any(n=='START' for n,_ in self.c.robot.client.commands))

    def test_alarm_preflight(self):
        self.c.robot.client.alarm=True
        with self.assertRaisesRegex(RuntimeError,'alarm'):
            self.start()
        self.assertFalse(any(n=='MOVL' for n,_ in self.c.robot.client.commands))

    def test_alarm_after_p002_write_blocks_start(self):
        original=self.c.robot.client.execute
        def alarm(name,data=''):
            result=original(name,data)
            if name=='LOADV' and data.startswith('4,2,'):
                self.c.robot.client.alarm=True
            return result
        with patch.object(self.c.robot.client,'execute',alarm):
            self.start()
            self.assertIn('alarm',self.finish()['error'])
        self.assertFalse(any(n=='START' for n,_ in self.c.robot.client.commands))

    def test_tcp_lost_after_movl_never_retries(self):
        self.c.robot.client.scenario='lost_movl_reply'
        self.start()
        result=self.finish()
        self.assertTrue(result['outcome_unknown'])
        self.assertIsNotNone(result['pending'])
        self.assertEqual(sum(n=='MOVL' for n,_ in self.c.robot.client.commands),1)

    def test_failed_tcp_telemetry_is_unknown(self):
        self.c.vacuum(1,True)
        self.c.robot.client.scenario='tcp_lost'
        with self.assertRaises(RuntimeError):
            self.c.refresh()
        snap=self.c.snapshot()
        self.assertFalse(snap['robot']['connected'])
        self.assertIsNone(snap['robot']['flags']['running'])
        self.assertIsNone(snap['robot']['laser'])
        self.assertEqual(snap['robot']['vacuum'],[None,None])

    def test_duplicate_start_and_stale_revision(self):
        with self.assertRaisesRegex(RuntimeError,'cũ'):
            self.c.start('request-0001',-1)
        self.start()
        with self.assertRaisesRegex(RuntimeError,'Duplicate'):
            self.start()
        with self.assertRaises(RuntimeError):
            self.start(request='request-0002')
        self.assertEqual(self.finish()['state'],'COMPLETE')

    def test_restart_between_intent_and_result_is_fault(self):
        self.c.journal.update(state='MOVE_XY')
        self.c.journal.intent('MOVL',{'x':1})
        journal=Journal(self.c.journal.path)
        self.assertEqual(journal.state['state'],'FAULT')
        self.assertTrue(journal.state['outcome_unknown'])
        self.assertEqual(journal.state['pending']['name'],'MOVL')

    def test_second_server_process_is_blocked(self):
        with self.assertRaisesRegex(RuntimeError,'process thứ hai'):
            Cycle(self.cfg,True,self.c.journal.path)

    def test_fault_does_not_restart_and_requires_inspection_ack(self):
        self.c.robot.client.scenario='lost_movl_reply'
        self.start()
        self.finish()
        with self.assertRaises(RuntimeError):
            self.start(request='request-0002')
        with self.assertRaisesRegex(RuntimeError,'kiểm tra'):
            self.c.acknowledge()
        self.c.acknowledge(inspected=True)
        self.assertEqual(self.c.journal.state['state'],'IDLE')
        self.assertTrue(self.c.robot.client.holding)  # No implicit HOLD release.

    def test_manual_vacuum_on_off(self):
        self.c.vacuum(1,True)
        self.assertEqual(self.c.robot.vacuum,[True,False])
        self.c.vacuum(1,False)
        self.assertEqual(self.c.robot.vacuum,[False,False])

    def test_complementary_hands_never_simultaneously_on(self):
        self.c.robot.vacuum_config['mode']='complementary'
        self.c.vacuum(1,True)
        self.assertEqual(self.c.robot.vacuum,[True,False])
        self.c.vacuum(2,True)
        self.assertEqual(self.c.robot.vacuum,[False,True])
        for name,data in self.c.robot.client.commands:
            if name=='IOWRITE':
                self.assertNotEqual(int(data.split(',')[2])&3,3)

    def test_real_hands_unknown_are_blocked(self):
        self.c.robot.vacuum_config={**self.cfg['vacuum'],'verified':False}
        with self.assertRaisesRegex(RuntimeError,'JOB'):
            self.c.vacuum(1,True)

    def test_inverted_hand_polarity(self):
        self.c.robot.vacuum_config['active_high']=[False,False]
        self.c.robot.vacuum_config['mode']='complementary'
        self.c.vacuum(1,True)
        self.assertEqual(self.c.robot.vacuum,[True,False])
        self.assertEqual(self.c.robot.client.byte&3,2)

    def test_lost_vacuum_reply_becomes_unknown(self):
        self.c.vacuum(1,True)
        original=self.c.robot.client.execute
        def lost(name,data=''):
            if name=='IOWRITE':
                from robot.protocol import OutcomeUnknown
                raise OutcomeUnknown('IOWRITE response lost')
            return original(name,data)
        with patch.object(self.c.robot.client,'execute',lost):
            with self.assertRaises(RuntimeError):
                self.c.vacuum(2,True)
        self.assertEqual(self.c.snapshot()['robot']['vacuum'],[None,None])
        self.assertEqual(self.c.journal.state['state'],'FAULT')

    def test_real_stop_reason_is_never_faked_by_config(self):
        from robot.job_control import stop_reason_proven
        self.c.robot.mock=False
        self.assertFalse(stop_reason_proven(self.c.robot,dict(running_seen=True,laser_edge=True)))

    def test_full_real_is_blocked_even_after_commissioning_flags(self):
        from workflow.safety_checks import commissioning_hash
        cfg=self.cfg
        cfg['motion'].update(expected_tool=0,expected_posture=0)
        cfg['laser']['verified']=True
        cfg['job'].update(verified=True,start_verified=True,min_z=0,max_descent_mm=500,speed_mm_s=5)
        for key in cfg['commissioning']:
            if isinstance(cfg['commissioning'][key],bool):
                cfg['commissioning'][key]=True
        cfg['commissioning']['evidence']='test evidence'
        cfg['commissioning']['config_sha256']=commissioning_hash(cfg)
        with self.assertRaisesRegex(RuntimeError,'adapter'):
            real_gate(cfg)

    def test_all_not_ready_flags_block(self):
        flags=dict(servo=True,remote=True,play=True,teach=False,alarm=False,error=False,running=False,hold=False)
        for key in flags:
            for bad in (None,not flags[key]):
                with self.subTest(key=key,bad=bad),self.assertRaises(RuntimeError):
                    ready({**flags,key:bad})

    def test_config_validation_and_durable_settings(self):
        with self.assertRaises(ValueError):
            self.c.configure({'movl_speed':float('nan')})
        self.c.configure({'z_safe':510,'rz':0})
        saved=json.loads((Path(self.tmp.name)/'operator_settings.json').read_text())
        self.assertEqual(saved['z_safe'],510)
        self.assertEqual(self.c.cfg['settings']['z_safe'],510)

    def test_same_xy_stack_is_allowed_on_second_cycle(self):
        sample=self.c.vision.sample
        def single_stack():
            frame,stamp,items=sample()
            return frame,stamp,[i for i in items if i['id']==1]
        with patch.object(self.c.vision,'sample',side_effect=single_stack):
            self.start()
            first=self.finish()
            self.assertEqual(first['state'],'COMPLETE',first['error'])
            self.c.acknowledge()
            self.c.robot.client.laser=False
            self.start(request='request-0002')
            result=self.finish()
            self.assertEqual(result['state'],'COMPLETE',result['error'])
            self.assertEqual(result['target']['xyz'][:2],first['target']['xyz'][:2])
            self.assertEqual(sum(n=='MOVL' for n,d in self.c.robot.client.commands),4)
            self.assertEqual([d for n,d in self.c.robot.client.commands if n=='START'],['MOVEUNTI','H2ON','DROP','H2OFF','MOVEUNTI','H2ON','DROP','H2OFF'])


if __name__=='__main__':
    unittest.main()
