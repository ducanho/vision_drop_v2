"""NX100 HW0483202 Integer I000: MockClient/FakeSocket only."""
import socket
import unittest
import tempfile
from pathlib import Path
from unittest.mock import patch, PropertyMock

from core.settings import load_config
from robot.controller import Controller
from robot.protocol import Client, OutcomeUnknown, encode_command
from tests.pickdrop_support import PickDropCase
from tests.test_protocol import FakeSocket, GREETING
from fastapi.testclient import TestClient
from app import create_app


class SpeedWireTests(unittest.TestCase):
    def test_exact_integer_wire_and_scalar_reply(self):
        sock=FakeSocket([GREETING,b'OK: LOADV\r\n',b'0000\r\n'])
        with patch('socket.create_connection',return_value=sock):
            Client('127.0.0.1').execute('LOADV','1,0,3500')
        self.assertEqual(sock.sent[-2:],[b'HOSTCTRL_REQUEST LOADV 9\r\n',b'1,0,3500\r'])
        sock=FakeSocket([GREETING,b'OK: SAVEV\r\n',b'3500\r'])
        robot=Controller(load_config(),False)
        with patch('socket.create_connection',return_value=sock):
            self.assertEqual(robot.read_i000(),3500)
        self.assertEqual(sock.sent[-2:],[b'HOSTCTRL_REQUEST SAVEV 4\r\n',b'1,0\r'])

    def test_whitelist_only_i000_and_requested_range(self):
        for value in (0,1,5000):
            encode_command('LOADV',f'1,0,{value}')
        for command,data in [('LOADV','1,1,1'),('SAVEV','1,1'),('LOADV','1,0,-1'),
                             ('LOADV','1,0,5001'),('LOADV','1,0,1.5'),('LOADV','1,0,NaN'),
                             ('LOADV','1,0,1,0'),('LOADV','0,0,100')]:
            with self.subTest(data=data),self.assertRaises(ValueError):
                encode_command(command,data)

    def test_read_parser_signed_integer_and_invalid_responses(self):
        robot=Controller(load_config(),True)
        for raw,value in [('0',0),('5000',5000),('32767',32767),('-32768',-32768),('+03500',3500)]:
            with patch.object(robot.client,'execute',return_value=raw):
                self.assertEqual(robot.read_i000(),value)
        for raw in ('','1,0,3500','1.0','32768','-32769','x','ERROR: 2100'):
            with self.subTest(raw=raw),patch.object(robot.client,'execute',return_value=raw),self.assertRaises(RuntimeError):
                robot.read_i000()

    def test_lost_loadv_reply_no_network_retry(self):
        sock=FakeSocket([GREETING,b'OK: LOADV\r\n',socket.timeout('lost')])
        with patch('socket.create_connection',return_value=sock) as connect:
            with self.assertRaises(OutcomeUnknown):
                Client('127.0.0.1').execute('LOADV','1,0,3500')
        connect.assert_called_once()


class SpeedWriteTests(PickDropCase):
    def writes(self):
        return [(n,d) for n,d in self.c.robot.client.commands if n=='LOADV']

    def test_save_boundaries_readback_actual_and_no_python_speed_reinterpretation(self):
        legacy=self.c.cfg['settings']['movl_speed']
        for value in (0,1,5000):
            self.c.configure({},speed_i000=value)
            self.assertEqual(self.c.robot.read_i000(),value)
            self.assertTrue(self.c.journal.state['speed_write']['verified'])
            self.assertEqual(self.c.journal.state['state'],'IDLE')
            self.c.refresh()
            self.assertEqual(self.c.snapshot()['robot']['speed_i000'],value)
        self.assertEqual(len(self.writes()),3)
        self.assertEqual(self.c.cfg['settings']['movl_speed'],legacy)
        self.assertNotIn('speed_i000',self.c.cfg['settings'])
        self.assertEqual(self.starts(),[])

    def test_invalid_request_values_never_write(self):
        for value in (None,True,False,1.0,1.5,-1,5001,'100',float('nan')):
            with self.subTest(value=value),self.assertRaises(ValueError):
                self.c.configure({},speed_i000=value)
        self.assertEqual(self.writes(),[])

    def test_unsafe_states_and_unknown_outcomes_block_write(self):
        for state in ('COMPLETE','HOLD','FAULT','PICKDROP_RUNNING','JOB_DISPATCHED'):
            self.c.journal.update(state=state)
            with self.assertRaises(RuntimeError):
                self.c.configure({},speed_i000=100)
        self.c.journal.update(state='IDLE',outcome_unknown=True)
        with self.assertRaises(RuntimeError):
            self.c.configure({},speed_i000=100)
        self.c.journal.update(outcome_unknown=False,pending=dict(name='START'))
        with self.assertRaises(RuntimeError):
            self.c.configure({},speed_i000=100)
        self.assertEqual(self.writes(),[])

    def test_connection_busy_and_live_flags_block_write(self):
        self.c.robot.connected=False
        with self.assertRaises(RuntimeError):
            self.c.configure({},speed_i000=100)
        self.c.robot.connected=True
        with patch.object(type(self.c),'busy',new_callable=PropertyMock,return_value=True):
            with self.assertRaises(RuntimeError):
                self.c.configure({},speed_i000=100)
        for flag in ('running','hold','alarm','error','remote','teach'):
            flags=self.c.robot.status()
            flags[flag]=not flags[flag]
            with patch.object(self.c.robot,'status',return_value=flags),self.assertRaises(RuntimeError):
                self.c.configure({},speed_i000=100)
        self.assertEqual(self.writes(),[])

    def test_servo_off_stopped_allows_variable_write(self):
        self.c.robot.client.servo=False
        self.c.configure({},speed_i000=200)
        self.assertEqual(self.c.robot.client.i000,200)

    def test_mismatch_fault_no_local_save_or_success(self):
        with patch.object(self.c.robot,'read_i000',return_value=99):
            with self.assertRaisesRegex(RuntimeError,'readback'):
                self.c.configure({'z_safe':510},speed_i000=100)
        self.assertEqual(len(self.writes()),1)
        self.assertEqual(self.c.journal.state['state'],'FAULT')
        self.assertTrue(self.c.journal.state['outcome_unknown'])
        self.assertEqual(self.c.cfg['settings']['z_safe'],500)
        self.assertFalse((self.path.parent/'operator_settings.json').exists())

    def test_write_reply_lost_is_not_retried_or_cleared_by_restart(self):
        self.c.robot.client.scenario='lost_i000_reply'
        with self.assertRaises(RuntimeError):
            self.c.configure({},speed_i000=100)
        self.assertEqual(len(self.writes()),1)
        self.assertEqual(self.c.journal.state['pending']['payload'],'1,0,100')
        self.assertEqual(self.c.journal.state['state'],'FAULT')
        from workflow.state_machine import Journal
        restored=Journal(self.path)
        self.assertTrue(restored.state['outcome_unknown'])
        self.assertEqual(restored.state['pending']['payload'],'1,0,100')

    def test_readback_timeout_faults_without_retry(self):
        with patch.object(self.c.robot,'read_i000',side_effect=TimeoutError('I000 readback lost')):
            with self.assertRaisesRegex(RuntimeError,'readback lost'):
                self.c.configure({},speed_i000=100)
        self.assertEqual(len(self.writes()),1)
        self.assertEqual(self.c.journal.state['state'],'FAULT')

    def test_other_settings_invalid_prevents_remote_write(self):
        with self.assertRaises(ValueError):
            self.c.configure({'z_safe':9999},speed_i000=100)
        with self.assertRaises(ValueError):
            self.c.configure({},drop_values={'board_thickness_mm':0},speed_i000=100)
        self.assertEqual(self.writes(),[])

    def test_monitor_reflects_pendant_changes_and_does_not_write(self):
        for value in (1200,2300):
            self.c.robot.client.i000=value
            self.c.refresh()
            self.assertEqual(self.c.snapshot()['robot']['speed_i000'],value)
        self.assertEqual(self.writes(),[])
        self.c.robot.connected=False
        self.c.telemetry['connected']=False
        self.assertIsNone(self.c.snapshot()['robot']['speed_i000'])

    def test_speed_monitor_failure_does_not_add_motion_read_error(self):
        with patch.object(self.c.robot,'read_i000',side_effect=RuntimeError('I000 unavailable')):
            data=self.c.refresh()
        self.assertIsNone(data['speed_i000'])
        self.assertIn('unavailable',data['speed_error'])
        self.assertEqual(data['read_errors'],{})


class SpeedApiTests(unittest.TestCase):
    def test_existing_save_api_requires_connection_and_returns_verified_controller_value(self):
        with tempfile.TemporaryDirectory() as tmp:
            app=create_app(mock=True,journal_path=Path(tmp)/'execution.json')
            with TestClient(app,base_url='http://localhost') as client:
                client.post('/api/control/claim',json={})
                def save(value,request):
                    state=client.get('/api/state').json()
                    return client.post('/api/config',json=dict(settings={},speed_i000=value,
                                       revision=state['revision'],request_id=request))
                self.assertEqual(save(100,'speed-api-offline').status_code,409)
                cycle=app.state.cycle
                cycle.robot.connect();cycle.refresh()
                response=save(1250,'speed-api-save-01')
                self.assertEqual(response.status_code,200,response.text)
                self.assertEqual(response.json()['robot']['speed_i000'],1250)
                self.assertEqual(cycle.robot.client.i000,1250)
                self.assertNotIn('speed_i000',response.json()['settings'])
                self.assertEqual(save(None,'speed-api-null-01').status_code,422)
                cycle.robot.client.scenario='lost_i000_reply'
                response=save(1300,'speed-api-fault-01')
                self.assertEqual(response.status_code,409,response.text)
                self.assertIn('SPEED I000',response.json()['detail'])
                self.assertEqual(cycle.journal.state['state'],'FAULT')
                self.assertEqual(sum(n=='LOADV' and d.startswith('1,0,') for n,d in cycle.robot.client.commands),2)
