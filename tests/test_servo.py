"""Manual SVON 0/1: mock controller/FakeSocket only."""
import socket
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch, PropertyMock

from fastapi.testclient import TestClient
from app import create_app
from core.settings import load_config
from robot.protocol import Client, OutcomeUnknown, encode_command
from tests.test_protocol import FakeSocket, GREETING
from workflow.cycle import Cycle
from workflow.safety_checks import servo_on_ready,servo_off_ready


class ServoTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        cfg=load_config()
        cfg['timeout_seconds']=.04
        cfg['motion']['poll_seconds']=.005
        self.c=Cycle(cfg,True,Path(self.tmp.name)/'execution.json')
        self.c.robot.connect()
        self.c.robot.client.servo=False
        self.c.refresh()

    def tearDown(self):
        self.c.close()
        self.tmp.cleanup()

    def controls(self):
        return [(n,d) for n,d in self.c.robot.client.commands if n in ('SVON','START','MOVL','HOLD','LOADV','IOWRITE')]

    def test_only_servo_on_and_verified_readback(self):
        self.assertTrue(self.c.snapshot()['capabilities']['servo_on']['allowed'])
        execute=self.c.robot.client.execute
        def verify_intent(name,data=''):
            if name=='SVON':
                self.assertEqual(self.c.journal.state['pending']['name'],'SVON')
                self.assertEqual(self.c.journal.state['state'],'SERVO_ENABLING')
            return execute(name,data)
        with patch.object(self.c.robot.client,'execute',side_effect=verify_intent):
            self.c.servo_on()
        self.assertEqual(self.controls(),[('SVON','1')])
        self.assertTrue(self.c.snapshot()['robot']['flags']['servo'])
        self.assertEqual(self.c.journal.state['state'],'IDLE')
        self.assertEqual(self.c.journal.state['servo_enable'],dict(accepted=True,verified=True))
        self.assertFalse(self.c.snapshot()['capabilities']['servo_on']['allowed'])
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],0)

    def test_already_on_does_not_send_again(self):
        self.c.servo_on()
        with self.assertRaises(RuntimeError): self.c.servo_on()
        self.assertEqual(self.controls(),[('SVON','1')])

    def test_flags_block_unknown_or_unsafe_activation(self):
        good=self.c.robot.status()
        for key in ('remote','play','teach','running','alarm','error','hold','servo'):
            for value in (None,not good[key]):
                with self.subTest(key=key,value=value),self.assertRaises(RuntimeError):
                    servo_on_ready({**good,key:value})

    def test_state_pending_busy_or_disconnected_blocks_activation(self):
        for state in ('COMPLETE','FAULT','HOLD','SYSTEM_CHECK','SERVO_ENABLING'):
            self.c.journal.update(state=state)
            with self.assertRaises(RuntimeError): self.c.servo_on()
        self.c.journal.update(state='IDLE',pending=dict(name='START'),outcome_unknown=False)
        with self.assertRaises(RuntimeError): self.c.servo_on()
        self.c.journal.update(pending=None,outcome_unknown=True)
        with self.assertRaises(RuntimeError): self.c.servo_on()
        self.c.journal.update(outcome_unknown=False)
        with patch.object(Cycle,'busy',new_callable=PropertyMock,return_value=True):
            with self.assertRaises(RuntimeError): self.c.servo_on()
        self.c.robot.connected=False
        with self.assertRaises(RuntimeError): self.c.servo_on()
        self.assertEqual(self.controls(),[])

    def test_fresh_status_overrides_cached_eligibility(self):
        self.assertTrue(self.c.snapshot()['capabilities']['servo_on']['allowed'])
        self.c.robot.client.holding=True
        with self.assertRaises(RuntimeError): self.c.servo_on()
        self.assertEqual(self.controls(),[])

    def test_readback_may_be_delayed_without_resending(self):
        execute=self.c.robot.client.execute
        reads=0
        def delayed(name,data=''):
            nonlocal reads
            answer=execute(name,data)
            if name=='SVON': self.c.robot.client.servo=False
            if name=='RSTATS' and any(n=='SVON' for n,d in self.c.robot.client.commands):
                reads+=1
                if reads>=2: self.c.robot.client.servo=True
            return answer
        with patch.object(self.c.robot.client,'execute',side_effect=delayed): self.c.servo_on()
        self.assertTrue(self.c.journal.state['servo_enable']['verified'])
        self.assertEqual(self.controls(),[('SVON','1')])

    def test_ack_without_servo_on_is_fault_not_success(self):
        execute=self.c.robot.client.execute
        def not_enabled(name,data=''):
            answer=execute(name,data)
            if name=='SVON': self.c.robot.client.servo=False
            return answer
        with patch.object(self.c.robot.client,'execute',side_effect=not_enabled):
            with self.assertRaisesRegex(RuntimeError,'EXSVON'): self.c.servo_on()
        self.assertEqual(self.c.journal.state['state'],'FAULT')
        self.assertFalse(self.c.journal.state['servo_enable']['verified'])
        self.assertEqual(self.controls(),[('SVON','1')])

    def test_lost_reply_is_unknown_and_never_retried(self):
        execute=self.c.robot.client.execute
        def lost(name,data=''):
            answer=execute(name,data)
            if name=='SVON': raise OutcomeUnknown('SVON reply lost')
            return answer
        with patch.object(self.c.robot.client,'execute',side_effect=lost):
            with self.assertRaises(OutcomeUnknown): self.c.servo_on()
        self.assertEqual(self.c.journal.state['state'],'FAULT')
        self.assertTrue(self.c.journal.state['outcome_unknown'])
        self.assertEqual(self.c.journal.state['pending']['name'],'SVON')
        self.assertEqual(self.controls(),[('SVON','1')])

    def test_hold_during_readback_is_not_overwritten(self):
        refresh=self.c.refresh
        def held():
            data=refresh()
            self.c.hold()
            return data
        with patch.object(self.c,'refresh',side_effect=held):
            # Avoid recursive refresh in hold(); emulate the admitted HOLD state.
            with patch.object(self.c,'hold',side_effect=lambda:self.c.journal.update(state='HOLD',outcome_unknown=True)):
                with self.assertRaises(RuntimeError): self.c.servo_on()
        self.assertEqual(self.c.journal.state['state'],'HOLD')
        self.assertEqual(self.controls(),[('SVON','1')])

    def test_restart_does_not_replay_enable(self):
        cfg=self.c.cfg
        path=self.c.journal.path
        self.c.journal.intent('SVON','1')
        self.c.journal.update(state='SERVO_ENABLING')
        self.c.close()
        self.c=Cycle(cfg,True,path)
        self.assertEqual(self.c.journal.state['state'],'FAULT')
        self.assertEqual(self.c.robot.client.commands,[])

    def test_wire_framing_and_lost_reply(self):
        sock=FakeSocket([GREETING,b'OK: SVON\r\n',b'0000\r\n'])
        with patch('socket.create_connection',return_value=sock):
            self.assertEqual(Client('127.0.0.1').execute('SVON','1'),'0000')
        self.assertEqual(sock.sent[-2:],[b'HOSTCTRL_REQUEST SVON 2\r\n',b'1\r'])
        sock=FakeSocket([GREETING,b'OK: SVON\r\n',socket.timeout('lost')])
        with patch('socket.create_connection',return_value=sock) as connect:
            with self.assertRaises(OutcomeUnknown): Client('127.0.0.1').execute('SVON','1')
        connect.assert_called_once()

    def test_malformed_servo_data_stays_blocked(self):
        for value in ('','2','01','1\r\n','0\r\n'):
            with self.assertRaises(ValueError): encode_command('SVON',value)

    def test_on_off_round_trip_uses_explicit_targets(self):
        self.c.servo_on()
        self.assertTrue(self.c.snapshot()['capabilities']['servo_off']['allowed'])
        self.c.servo_off()
        self.assertFalse(self.c.snapshot()['robot']['flags']['servo'])
        self.assertFalse(self.c.journal.state['servo_target'])
        self.assertTrue(self.c.journal.state['servo_enable']['verified'])
        self.assertTrue(self.c.snapshot()['capabilities']['servo_on']['allowed'])
        self.assertFalse(self.c.snapshot()['capabilities']['servo_off']['allowed'])
        self.assertEqual(self.controls(),[('SVON','1'),('SVON','0')])
        with self.assertRaises(RuntimeError): self.c.servo_off()
        self.assertEqual(self.controls(),[('SVON','1'),('SVON','0')])

    def test_servo_off_guards_running_hold_alarm_and_unknown(self):
        self.c.robot.client.servo=True
        good=self.c.robot.status()
        for key,value in [('running',True),('hold',True),('alarm',True),('servo',None),('remote',False)]:
            with self.subTest(key=key),self.assertRaises(RuntimeError):
                servo_off_ready({**good,key:value})
        self.c.robot.client.running=True
        with self.assertRaises(RuntimeError): self.c.servo_off()
        self.assertEqual(self.controls(),[])

    def test_servo_off_readback_timeout_never_retries(self):
        self.c.robot.client.servo=True
        execute=self.c.robot.client.execute
        def unchanged(name,data=''):
            answer=execute(name,data)
            if name=='SVON': self.c.robot.client.servo=True
            return answer
        with patch.object(self.c.robot.client,'execute',side_effect=unchanged):
            with self.assertRaisesRegex(RuntimeError,'SERVO OFF'): self.c.servo_off()
        self.assertEqual(self.c.journal.state['state'],'FAULT')
        self.assertFalse(self.c.journal.state['servo_enable']['verified'])
        self.assertEqual(self.controls(),[('SVON','0')])

    def test_lost_servo_off_reply_is_unknown(self):
        self.c.robot.client.servo=True
        execute=self.c.robot.client.execute
        def lost(name,data=''):
            answer=execute(name,data)
            if name=='SVON': raise OutcomeUnknown('SVON OFF reply lost')
            return answer
        with patch.object(self.c.robot.client,'execute',side_effect=lost):
            with self.assertRaises(OutcomeUnknown): self.c.servo_off()
        self.assertTrue(self.c.journal.state['outcome_unknown'])
        self.assertEqual(self.c.journal.state['pending']['payload'],'0')
        self.assertEqual(self.controls(),[('SVON','0')])

    def test_servo_off_wire_format(self):
        sock=FakeSocket([GREETING,b'OK: SVON\r\n',b'0000\r\n'])
        with patch('socket.create_connection',return_value=sock):
            self.assertEqual(Client('127.0.0.1').execute('SVON','0'),'0000')
        self.assertEqual(sock.sent[-2:],[b'HOSTCTRL_REQUEST SVON 2\r\n',b'0\r'])

    def test_non_boolean_servo_target_is_rejected(self):
        for value in (None,0,1,'0','1'):
            with self.subTest(value=value),self.assertRaises(ValueError): self.c.set_servo(value)
        self.assertEqual(self.controls(),[])


class ServoApiTests(unittest.TestCase):
    def test_lease_revision_deduplication_and_no_implicit_commands(self):
        with tempfile.TemporaryDirectory() as tmp:
            app=create_app(mock=True,journal_path=Path(tmp)/'execution.json')
            with TestClient(app,base_url='http://localhost') as client:
                c=app.state.cycle
                self.assertEqual(c.robot.client.commands,[])
                body=dict(request_id='servo-api-request-001',revision=c.journal.state['revision'])
                self.assertEqual(client.post('/api/robot/servo-on',json=body).status_code,423)
                client.post('/api/control/claim',json={})
                c.robot.connect()
                c.robot.client.servo=False
                c.refresh()
                self.assertEqual(client.post('/api/robot/servo-on',json={**body,'revision':-1}).status_code,409)
                response=client.post('/api/robot/servo-on',json=body)
                self.assertEqual(response.status_code,200,response.text)
                self.assertTrue(response.json()['servo_enable']['verified'])
                body['revision']=c.journal.state['revision']
                self.assertEqual(client.post('/api/robot/servo-on',json=body).status_code,409)
                controls=[(n,d) for n,d in c.robot.client.commands if n in ('SVON','START','MOVL','HOLD')]
                self.assertEqual(controls,[('SVON','1')])
                off_body=dict(request_id='servo-api-off-001',revision=c.journal.state['revision'])
                self.assertEqual(client.post('/api/robot/servo-off',json={**off_body,'revision':-1}).status_code,409)
                response=client.post('/api/robot/servo-off',json=off_body)
                self.assertEqual(response.status_code,200,response.text)
                self.assertFalse(response.json()['robot']['flags']['servo'])
                self.assertTrue(response.json()['servo_enable']['verified'])
                off_body['revision']=c.journal.state['revision']
                self.assertEqual(client.post('/api/robot/servo-off',json=off_body).status_code,409)
                controls=[(n,d) for n,d in c.robot.client.commands if n in ('SVON','START','MOVL','HOLD')]
                self.assertEqual(controls,[('SVON','1'),('SVON','0')])
