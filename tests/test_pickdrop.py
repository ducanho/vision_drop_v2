"""PICKDROP transaction and completion evidence, using MockClient/FakeSocket only."""
import socket
import time
from unittest.mock import patch

from core.settings import AXES, load_config
from robot.controller import Controller
from robot.protocol import Client, OutcomeUnknown, encode_command
from tests.pickdrop_support import PickDropCase
from tests.test_protocol import FakeSocket, GREETING
from workflow.state_machine import Journal
from workflow.safety_checks import real_gate
from workflow.pickdrop import real_blockers


class PickDropTests(PickDropCase):
    def test_single_start_both_variables_verified_before_start_and_no_pc_motion(self):
        with patch.object(self.c, '_job', side_effect=AssertionError('legacy JOB')), \
             patch.object(self.c, '_wait_vacuum_ready', side_effect=AssertionError('PC vacuum')):
            result = self.run_cycle()
        self.assertEqual(result['state'], 'COMPLETE', result['error'])
        commands = self.c.robot.client.commands
        self.assertEqual(self.starts(), ['PICKDROP'])
        start = commands.index(('START','PICKDROP'))
        for index in ('2', '10'):
            self.assertTrue(any(n == 'LOADV' and d.startswith('4,'+index+',') for n,d in commands[:start]))
            self.assertIn(('SAVEV','4,'+index), commands[:start])
        self.assertFalse(any(n in ('MOVL', 'IOWRITE') for n,d in commands))
        self.assertTrue(result['pickdrop']['reset_confirmed'])
        self.assertTrue(result['pickdrop']['completed'])
        self.assertFalse(result['drop_release']['board_release_confirmed'])
        self.assertEqual(self.c.stack.snapshot()['stacked_count'], 1)

    def test_wrong_p002_blocks_start(self):
        execute = self.c.robot.client.execute
        def wrong(name, data=''):
            result = execute(name, data)
            return result.replace('850.000', '851.000') if (name,data)==('SAVEV','4,2') else result
        with patch.object(self.c.robot.client, 'execute', side_effect=wrong):
            result = self.run_cycle()
        self.assert_fault(result)
        self.assertEqual(self.starts(), [])

    def test_wrong_p010_blocks_start(self):
        with patch.object(self.c.robot.client, 'p010', ['1','0','0','0','999','0','0','0','0','0']):
            execute = self.c.robot.client.execute
            def wrong(name, data=''):
                result = execute(name,data)
                return '1,0,0,0,999,0,0,0,0,0' if (name,data)==('SAVEV','4,10') else result
            with patch.object(self.c.robot.client, 'execute', side_effect=wrong):
                result = self.run_cycle()
        self.assert_fault(result)
        self.assertEqual(self.starts(), [])

    def test_p010_rechecked_after_write_before_start(self):
        prepare = self.c._prepare_drop_variable
        def changed():
            prepare()
            self.c.robot.client.p010[2] = '999'
        with patch.object(self.c, '_prepare_drop_variable', side_effect=changed):
            result = self.run_cycle()
        self.assert_fault(result)
        self.assertEqual(self.starts(), [])

    def test_second_start_for_same_cycle_is_blocked_even_after_ack(self):
        self.c.journal.update(cycle_id='same-cycle')
        self.c._prepare_pick_variable(dict(zip(AXES,(850,100,500,180,0,90))))
        self.c._prepare_drop_variable()
        self.c._start_pickdrop()
        with self.assertRaisesRegex(RuntimeError, 'không gửi lại'):
            self.c._start_pickdrop()
        self.assertEqual(self.starts(), ['PICKDROP'])

    def test_b091_regression_after_success_is_unknown(self):
        with patch.object(self.c.robot, 'read_b091', side_effect=[0,1,0]):
            result = self.run_cycle()
        self.assert_fault(result)
        self.assertTrue(result['outcome_unknown'])
        self.assertIn('reset lại', result['error'])

    def test_second_vacuum_sensor_alone_satisfies_mock_internal_or(self):
        self.c.robot.client.scenario = 'vacuum_sensor_2'
        result = self.run_cycle()
        self.assertEqual(result['state'], 'COMPLETE', result['error'])
        self.assertEqual(self.starts(), ['PICKDROP'])

    def test_receipt_cannot_cross_cycle_or_connection(self):
        self.c.journal.update(cycle_id='original')
        self.c._prepare_pick_variable(dict(zip(AXES,(850,100,500,180,0,90))))
        self.c._prepare_drop_variable()
        self.c.robot.connect()
        with self.assertRaises(RuntimeError):
            self.c._start_pickdrop()
        self.c.journal.update(cycle_id='another')
        with self.assertRaises(RuntimeError):
            self.c._start_pickdrop()
        self.assertEqual(self.starts(), [])

    def test_pending_zero_and_running_off_do_not_complete(self):
        self.c.cfg['drop']['job_timeout'] = .08
        with patch.object(self.c.robot, 'read_b091', return_value=0), patch.object(self.c.robot, 'status',
                return_value=dict(servo=True,remote=True,play=True,teach=False,alarm=False,error=False,hold=False,running=False)):
            result = self.run_cycle()
        self.assert_fault(result)
        self.assertTrue(result['outcome_unknown'])

    def test_success_checkpoint_while_running_does_not_count(self):
        self.c.cfg['drop']['job_timeout'] = .08
        with patch.object(self.c.robot, 'read_b091', side_effect=[0]+[1]*100):
            result = self.run_cycle()
        self.assert_fault(result)
        self.assertFalse(result['pickdrop']['completed'])

    def test_old_success_without_post_start_reset_does_not_count(self):
        with patch.object(self.c.robot, 'read_b091', return_value=1):
            result = self.run_cycle()
        self.assert_fault(result)
        self.assertIn('reset', result['error'])

    def test_fault_checkpoint_no_count_no_hand_off_no_restart(self):
        self.c.robot.client.scenario = 'pickdrop_fault'
        result = self.run_cycle()
        self.assert_fault(result)
        self.assertIn('B091=2', result['error'])
        self.assertEqual(self.starts(), ['PICKDROP'])
        self.assertTrue(result['outcome_unknown'])
        with self.assertRaises(RuntimeError):
            self.c.start('another-request', result['revision'])
        with self.assertRaises(RuntimeError):
            self.c.acknowledge()

    def test_invalid_b091_no_count(self):
        with patch.object(self.c.robot, 'read_b091', return_value=255):
            result = self.run_cycle()
        self.assert_fault(result)
        self.assertIn('UNKNOWN', result['error'])

    def test_lost_start_reply_never_retries_or_counts(self):
        self.c.robot.client.scenario = 'lost_pickdrop_reply'
        result = self.run_cycle()
        self.assert_fault(result)
        self.assertEqual(self.starts(), ['PICKDROP'])
        self.assertEqual(result['pending']['payload'], 'PICKDROP')
        self.assertTrue(result['outcome_unknown'])
        self.assertTrue(Journal(self.path).state['outcome_unknown'])

    def test_communication_timeout_no_count(self):
        self.c.robot.client.scenario = 'pickdrop_disconnect'
        result = self.run_cycle()
        self.assert_fault(result)
        self.assertTrue(result['outcome_unknown'])
        self.assertEqual(self.starts(), ['PICKDROP'])

    def test_hold_mid_cycle_cannot_resume(self):
        sample = self.c._sample_pickdrop_result
        def held():
            value = sample()
            self.c.hold()
            return value
        with patch.object(self.c, '_sample_pickdrop_result', side_effect=held):
            result = self.run_cycle()
        self.assertEqual(result['state'], 'HOLD')
        self.assertEqual(self.c.stack.snapshot()['stacked_count'], 0)
        self.assertNotIn(('HOLD','0'), self.c.robot.client.commands)
        self.assertEqual(self.starts(), ['PICKDROP'])

    def test_alarm_mid_cycle_no_count(self):
        sample = self.c._sample_pickdrop_result
        def alarm():
            value = sample()
            self.c.robot.client.alarm = True
            return value
        with patch.object(self.c, '_sample_pickdrop_result', side_effect=alarm):
            result = self.run_cycle()
        self.assert_fault(result)

    def test_job_mismatch_no_count(self):
        start = self.c._start_pickdrop
        def wrong_job():
            start()
            self.c.robot.client.last_job = 'DROP'
        with patch.object(self.c, '_start_pickdrop', side_effect=wrong_job):
            result = self.run_cycle()
        self.assert_fault(result)
        self.assertIn('RJSEQ', result['error'])

    def test_reconnect_mid_job_no_count(self):
        start = self.c._start_pickdrop
        def reconnect():
            start()
            self.c.robot.connect()
        with patch.object(self.c, '_start_pickdrop', side_effect=reconnect):
            result = self.run_cycle()
        self.assert_fault(result)
        self.assertIn('connection', result['error'])

    def test_final_position_mismatch_no_count(self):
        self.c.robot.client.scenario = 'pickdrop_bad_pose'
        self.assert_fault(self.run_cycle())

    def test_mock_final_pose_uses_p011_translation_not_hardcoded_z200(self):
        self.c.robot.client.p011[2:5] = ['10','-20','150']
        result = self.run_cycle()
        self.assertEqual(result['state'], 'COMPLETE', result['error'])
        self.assertEqual(self.c.robot.client.pose[:3], [710,-820,480])
        self.assertEqual(result['post_drop_target']['z'], 480)

    def test_count_once_and_restart_never_replays(self):
        result = self.run_cycle()
        self.assertEqual(result['state'], 'COMPLETE', result['error'])
        self.c._confirm_pickdrop()
        self.assertEqual(self.c.stack.snapshot()['stacked_count'], 1)
        self.assertEqual(self.starts(), ['PICKDROP'])
        self.c.close()
        from workflow.cycle import Cycle
        self.c = Cycle(self.cfg,True,self.path)
        self.assertEqual(self.c.robot.client.commands, [])
        self.assertEqual(self.c.stack.snapshot()['last_picked_id'], 1)
        self.assertEqual(self.c.stack.snapshot()['last_completed_drop_cycle_id'], 'pickdrop-001')

    def test_restart_in_job_or_after_completion_before_count_requires_inspection(self):
        self.c.journal.update(state='PICKDROP_RUNNING', cycle_id='interrupted')
        result = Journal(self.path).state
        self.assertEqual(result['state'], 'FAULT')
        self.assertTrue(result['outcome_unknown'])
        self.c.journal.update(state='PICKDROP_COMPLETE')
        self.assertTrue(Journal(self.path).state['outcome_unknown'])

    def test_real_gate_blocks_unconfirmed_frame_even_with_old_override(self):
        self.c.cfg['workflow']['end_after_moveunti_start'] = True
        self.c.cfg['pickdrop']['p011_frame']=None
        self.c.mock = False
        self.assertFalse(self.c.snapshot()['capabilities']['full_cycle']['allowed'])
        with self.assertRaisesRegex(RuntimeError, r'IMOV P011'):
            real_gate(self.c.cfg)
        self.assertTrue(any('IMOV P011' in issue for issue in real_blockers(self.c.cfg)))
        self.assertEqual(self.starts(), [])

    def test_operator_confirmed_real_contract_allows_real_path_using_mock_client(self):
        # Switch workflow mode only. Underlying client remains offline MockClient.
        self.c.mock=False
        self.assertTrue(self.c.snapshot()['capabilities']['full_cycle']['allowed'])
        with patch('socket.create_connection',side_effect=AssertionError('No real robot')):
            result=self.run_cycle()
        self.assertEqual(result['state'],'COMPLETE',result['error'])
        self.assertEqual(self.starts(),['PICKDROP'])
        self.assertEqual(result['post_drop_target']['z'],530)

    def test_changed_job_export_relocks_real(self):
        self.c.cfg['pickdrop']['job_sha256']='changed'
        with self.assertRaises(RuntimeError):
            real_gate(self.c.cfg)

    def test_real_path_reads_actual_base_p011_vector(self):
        self.c.mock=False
        self.c.robot.client.p011[2:5]=['0','0','150']
        result=self.run_cycle()
        self.assertEqual(result['state'],'COMPLETE',result['error'])
        self.assertEqual(result['post_drop_target']['z'],480)

    def test_rotation_in_p011_blocks_start_rather_than_assuming_z_lift(self):
        self.c.mock=False
        self.c.robot.client.p011[5]='10'
        self.assert_fault(self.run_cycle())
        self.assertEqual(self.starts(),[])

    def test_pending_or_unknown_idle_cannot_start(self):
        for values in (dict(outcome_unknown=True),dict(outcome_unknown=False,pending={'name':'START'})):
            self.c.journal.update(state='IDLE', **values)
            with self.assertRaisesRegex(RuntimeError, 'ACK'):
                self.c.start('pending-request', self.c.journal.state['revision'])
        self.assertEqual(self.starts(), [])

    def test_motion_jobs_cannot_be_manual_hand_jobs(self):
        from robot.vacuum import validate_mapping
        cfg=load_config()['vacuum']
        for name in ('MOVEUNTI','DROP','PICKDROP','HOME'):
            with self.assertRaises(RuntimeError):
                validate_mapping({**cfg,'jobs':{**cfg['jobs'],'2_on':name}})


class PickDropWireTests(PickDropCase):
    def test_b091_wire_format_and_byte_writes_rejected(self):
        robot = Controller(load_config(),False)
        for value in (0,1,2,255):
            sock = FakeSocket([GREETING,b'OK: SAVEV\r\n', str(value).encode()+b'\r'])
            with patch('socket.create_connection',return_value=sock):
                self.assertEqual(robot.read_b091(), value)
            self.assertEqual(sock.sent[-2:], [b'HOSTCTRL_REQUEST SAVEV 5\r\n',b'0,91\r'])
        for value in ('','-1','256','1.0','0,91,1','x'):
            with patch.object(robot.client,'execute',return_value=value), self.assertRaises(RuntimeError):
                robot.read_b091()
        for data in ('0,90,0','0,91,1','2,0,100'):
            with self.assertRaises(ValueError):
                encode_command('LOADV', data)

    def test_pickdrop_start_wire_and_no_retry_on_lost_reply(self):
        sock = FakeSocket([GREETING,b'OK: START\r\n',socket.timeout('lost')])
        with patch('socket.create_connection',return_value=sock) as connect:
            with self.assertRaises(OutcomeUnknown):
                Client('127.0.0.1',allowed_start_jobs=('PICKDROP',)).execute('START','PICKDROP')
        connect.assert_called_once()
        self.assertEqual(sock.sent[-2:], [b'HOSTCTRL_REQUEST START 9\r\n',b'PICKDROP\r'])

    def test_mock_never_opens_socket(self):
        with patch('socket.create_connection',side_effect=AssertionError('Real network forbidden')):
            self.assertEqual(self.run_cycle()['state'], 'COMPLETE')
