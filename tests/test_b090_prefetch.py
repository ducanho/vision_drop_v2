"""B090 and overlapped preparation: simulated hardware only."""
import socket
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from robot.controller import Controller
from robot.protocol import Client, ProtocolError, OutcomeUnknown, encode_command
from core.settings import load_config
from tests.test_protocol import FakeSocket, GREETING
from workflow.cycle import Cycle


def board():
    return dict(id=2, instance=2, xyz=(800., 100., 0.), confidence=.95)


class ByteReadTests(unittest.TestCase):
    def test_wire_zero_one_and_upper_boundary(self):
        for value in (0, 1, 255):
            with self.subTest(value=value):
                sock=FakeSocket([GREETING, b'OK: SAVEV\r\n', str(value).encode()+b'\r'])
                robot=Controller(load_config(), False)
                with patch('socket.create_connection', return_value=sock):
                    self.assertEqual(robot.read_b090(), value)
                self.assertEqual(sock.sent[-2:], [b'HOSTCTRL_REQUEST SAVEV 5\r\n', b'0,90\r'])

    def test_read_timeout_is_not_control_outcome_unknown_and_no_retry(self):
        sock=FakeSocket([GREETING, b'OK: SAVEV\r\n', socket.timeout('lost')])
        with patch('socket.create_connection', return_value=sock) as connect:
            with self.assertRaises(ProtocolError) as caught:
                Client('127.0.0.1').execute('SAVEV', '0,90')
        self.assertNotIsInstance(caught.exception, OutcomeUnknown)
        connect.assert_called_once()

    def test_invalid_byte_replies(self):
        robot=Controller(load_config(), True)
        for value in ('', '-1', '256', '1.0', '0,90,1', 'ERROR: 2100', 'x'):
            with self.subTest(value=value), patch.object(robot.client, 'execute', return_value=value):
                with self.assertRaises(RuntimeError):
                    robot.read_b090()

    def test_only_b090_read_is_added_and_byte_writes_stay_blocked(self):
        for command, data in [('SAVEV','0,89'), ('SAVEV','0,92'), ('LOADV','0,90,1')]:
            with self.assertRaises(ValueError):
                encode_command(command, data)


class CheckpointTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        cfg=load_config()
        cfg['vision'].update(stable_frames=3, stable_seconds=.04)
        cfg['motion'].update(poll_seconds=.005, target_timeout=.08, settle_samples=2)
        self.c=Cycle(cfg, True, Path(self.tmp.name)/'execution.json')
        self.c.robot.connect()
        self.c.journal.update(cycle_id='drop-current', automatic_repeat=True, target=board())
        self.c.robot.client.last_job='PICKDROP'

    def tearDown(self):
        self.c.close()
        self.tmp.cleanup()

    def trigger(self):
        with patch.object(self.c.robot, 'read_b090', side_effect=[0,1]):
            self.c._arm_drop_prefetch()
            self.c._poll_drop_prefetch()

    def prepare(self):
        self.trigger()
        now=time.monotonic()
        v=self.c.vision
        v._prefetch['started_at']=now-.2
        for i in range(4):
            v._update_next_pick(i+1, now-.12+i*.03, [board()])
        v.connected=True
        v.stamp=now-.03
        self.c.stack.confirm('drop-current', picked_id=2)
        self.c.journal.update(cycle_id='next-cycle')

    def fresh(self):
        time.sleep(.003)
        return 100, time.monotonic(), [board()]

    def test_old_high_never_triggers_without_reset(self):
        with patch.object(self.c.robot, 'read_b090', return_value=1):
            self.c._arm_drop_prefetch()
            for _ in range(3):
                self.c._poll_drop_prefetch()
        self.assertFalse(self.c.prefetch['reset_confirmed'])
        self.assertIsNone(self.c.vision._next_target)

    def test_valid_reset_then_high_triggers_once(self):
        with patch.object(self.c.vision, 'prepare_next_pick', wraps=self.c.vision.prepare_next_pick) as prepare:
            self.trigger()
            self.c._poll_drop_prefetch()
            prepare.assert_called_once_with(after_id=2, cycle_id='drop-current')
        self.assertEqual(self.c.vision.prefetch_status()['state'], 'DETECTING')
        self.assertFalse(any(n=='MOVL' for n,d in self.c.robot.client.commands))

    def test_read_failure_invalidates_without_stopping_drop(self):
        for error in (TimeoutError('timeout'), RuntimeError('parse'), ProtocolError('lost')):
            with patch.object(self.c.robot, 'read_b090', side_effect=error):
                self.c._arm_drop_prefetch()
            self.assertEqual(self.c.prefetch['state'], 'INVALID')
            self.assertIsNone(self.c.vision._next_target)

    def test_missing_reset_finishes_drop_and_falls_back_to_full_stability(self):
        self.c._prepare_drop_variable()
        self.c._prepare_pick_variable(dict(x=850,y=100,z=500,rx=180,ry=0,rz=90))
        with patch.object(self.c.robot,'read_b090',return_value=1):
            self.c._start_pickdrop()
            self.c._arm_drop_prefetch()
            self.c._wait_pickdrop_complete()
            self.c._confirm_pickdrop()
        self.assertEqual(self.c.journal.state['state'],'COMPLETE')
        self.assertEqual(self.c.prefetch['state'],'INVALID')
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],1)
        self.assertEqual(self.c.robot.pose()[0]['z'],530)
        frame=100
        def frames():
            nonlocal frame
            time.sleep(.003)
            frame+=1
            return frame,time.monotonic(),[board()]
        with patch.object(self.c.vision,'sample',side_effect=frames):
            self.c._target()
        self.assertEqual(self.c.journal.state['target_prefilled_frames'],0)
        self.assertGreaterEqual(self.c.journal.state['target_wait_seconds'],.04)

    def test_wrong_job_cycle_or_connection_cannot_trigger(self):
        for kind in ('job','cycle','reconnect','disconnect'):
            with self.subTest(kind=kind):
                self.c.robot.client.last_job='PICKDROP'
                self.c.robot.connected=True
                with patch.object(self.c.robot,'read_b090',return_value=0):
                    self.c._arm_drop_prefetch()
                if kind=='job': self.c.robot.client.last_job='OTHER'
                if kind=='cycle': self.c.journal.update(cycle_id='different')
                if kind=='reconnect': self.c.robot.connect()
                if kind=='disconnect': self.c.robot.connected=False
                with patch.object(self.c.robot,'read_b090',return_value=1):
                    self.c._poll_drop_prefetch()
                self.assertEqual(self.c.prefetch['state'],'INVALID')
                self.assertIsNone(self.c.vision._next_target)

    def test_empty_or_unstable_frames_never_claim_ready(self):
        self.trigger()
        v=self.c.vision
        v._prefetch['started_at']=time.monotonic()-.1
        v._update_next_pick(1, time.monotonic(), [])
        self.assertEqual(v.prefetch_status()['state'],'STABILIZING')
        v._update_next_pick(2, time.monotonic()+.001, [board()])
        self.assertNotEqual(v.prefetch_status()['state'],'READY')
        self.assertFalse(any(n=='MOVL' for n,d in self.c.robot.client.commands))

    def test_pretrigger_frames_are_excluded(self):
        self.trigger()
        v=self.c.vision
        v._update_next_pick(100,v._prefetch['started_at']-.001,[board()])
        self.assertEqual(v._next_target.samples,[])

    def test_ready_history_same_xy_and_id_can_be_reused_after_lift(self):
        self.prepare()
        self.assertEqual(self.c.vision.prefetch_status()['state'],'READY')
        with patch.object(self.c.vision,'sample',side_effect=AssertionError('Ready coordinates must not request another frame')):
            pose=self.c._target()
        self.assertEqual(pose['x'],800)
        self.assertEqual(self.c.journal.state['target']['prefetch_cycle_id'],'drop-current')
        self.assertGreaterEqual(self.c.journal.state['target_prefilled_frames'],3)
        self.assertFalse(any(n=='MOVL' for n,d in self.c.robot.client.commands))

    def test_ready_coordinates_remain_latched_without_age_or_scene_recheck(self):
        self.prepare()
        saved=self.c.vision.prefetch_status()['target']
        self.c.vision._update_next_pick(500,time.monotonic(),[])
        self.assertEqual(self.c.vision.prefetch_status()['state'],'READY')
        with patch('vision.pipeline.time.monotonic',return_value=time.monotonic()+10):
            target=self.c.vision.take_ready_pick(after_id=2,cycle_id='drop-current')
        self.assertEqual(target['pose'],saved['pose'])
        self.assertIsNone(self.c.vision.take_ready_pick(after_id=2,cycle_id='drop-current'))

    def test_stale_history_is_discarded(self):
        self.prepare()
        self.c.vision._next_target.last_stamp=time.monotonic()-self.c.cfg['vision']['max_frame_age']-1
        self.assertIsNone(self.c.vision.take_next_pick(after_id=2))
        self.assertEqual(self.c.vision.prefetch_status()['reason'],'stale target')

    def test_config_and_calibration_changes_discard_history(self):
        self.prepare()
        self.c.cfg['settings']['plane_z_mm']+=1
        self.assertIsNone(self.c.vision.take_next_pick(after_id=2))
        self.c.journal.update(cycle_id='drop-current')
        self.prepare()
        self.c.vision.calibration.T[0,3]+=1
        self.assertIsNone(self.c.vision.take_next_pick(after_id=2))

    def test_config_change_during_detection_invalidates(self):
        self.trigger()
        self.c.cfg['settings']['plane_z_mm']+=1
        self.c.vision._update_next_pick(10,time.monotonic(),[board()])
        self.assertEqual(self.c.vision.prefetch_status()['state'],'INVALID')

    def test_camera_loss_after_validation_blocks_dispatch(self):
        self.prepare()
        with patch.object(self.c.vision,'sample',side_effect=self.fresh):
            target=self.c._target()
        self.c.vision.connected=False
        with self.assertRaises(RuntimeError):
            self.c._prepare_pick_variable(target)
        self.assertFalse(any(n=='MOVL' for n,d in self.c.robot.client.commands))

    def test_config_change_after_validation_blocks_dispatch(self):
        self.prepare()
        with patch.object(self.c.vision,'sample',side_effect=self.fresh):
            target=self.c._target()
        self.c.cfg['settings']['plane_z_mm']+=1
        with patch.object(self.c.vision,'sample',side_effect=self.fresh), self.assertRaises(RuntimeError):
            self.c._prepare_pick_variable(target)
        self.assertFalse(any(n=='MOVL' for n,d in self.c.robot.client.commands))

    def test_prepared_history_can_only_be_taken_once(self):
        self.prepare()
        self.assertIsNotNone(self.c.vision.take_next_pick(after_id=2))
        self.c.vision.consume_next_pick()
        self.assertEqual(self.c.vision.prefetch_status()['state'],'CONSUMED')
        self.assertIsNone(self.c.vision.take_next_pick(after_id=2))

    def test_hold_alarm_disconnect_or_running_blocks_next_move(self):
        self.prepare()
        with patch.object(self.c.vision,'sample',side_effect=self.fresh):
            target=self.c._target()
        for kind in ('holding','alarm','running','disconnected'):
            with self.subTest(kind=kind):
                self.c.robot.client.holding=self.c.robot.client.alarm=self.c.robot.client.running=False
                self.c.robot.connected=True
                if kind=='disconnected': self.c.robot.connected=False
                else: setattr(self.c.robot.client,kind,True)
                with patch.object(self.c.vision,'sample',side_effect=self.fresh), self.assertRaises(RuntimeError):
                    self.c._prepare_pick_variable(target)
        self.assertFalse(any(n=='MOVL' for n,d in self.c.robot.client.commands))

    def test_restart_has_no_prepared_target_or_b090_evidence(self):
        self.prepare()
        cfg=self.c.cfg
        path=self.c.journal.path
        self.c.close()
        self.c=Cycle(cfg,True,path)
        self.assertIsNone(self.c.vision._next_target)
        self.assertEqual(self.c.prefetch['state'],'IDLE')
        self.assertEqual(self.c.robot.client.commands,[])


class OverlapTests(unittest.TestCase):
    def test_ready_during_drop_and_next_move_only_after_verified_lift(self):
        with tempfile.TemporaryDirectory() as tmp:
            cfg=load_config()
            cfg['vision'].update(stable_frames=2,stable_seconds=.04)
            cfg['motion'].update(poll_seconds=.01,settle_samples=2,target_timeout=2)
            cfg['workflow']['empty_seconds']=.1
            c=Cycle(cfg,True,Path(tmp)/'execution.json')
            try:
                c.connect()
                deadline=time.monotonic()+2
                while not c.vision.status()['connected'] and time.monotonic()<deadline:
                    time.sleep(.01)
                sample=c.vision.sample
                update=c.vision._update_next_pick
                def next_scene(frame,stamp,items):
                    return update(frame,stamp,[] if c.stack.snapshot()['stacked_count']>=1 else items)
                execute=c.robot.client.execute
                def slow_drop(name,data=''):
                    result=execute(name,data)
                    if name=='START' and data=='PICKDROP':
                        c.robot.client.deadline=time.monotonic()+.5
                    return result
                def observe():
                    if c.stack.snapshot()['stacked_count']==1:
                        raise AssertionError('No camera sample is allowed for the prepared second pick')
                    frame,stamp,items=sample()
                    return frame,stamp,[] if c.stack.snapshot()['stacked_count']>=2 else items
                with patch.object(c.vision,'sample',side_effect=observe), \
                     patch.object(c.vision,'_update_next_pick',side_effect=next_scene), \
                     patch.object(c.robot.client,'execute',side_effect=slow_drop), \
                     self.assertLogs('cycle',level='INFO') as log:
                    c.start('overlap-request-001',c.journal.state['revision'])
                    c.thread.join(10)
                self.assertFalse(c.busy)
                self.assertEqual(c.journal.state['state'],'COMPLETE',c.journal.state['error'])
                events='\n'.join(log.output)
                self.assertLess(events.index('NEXT_TARGET_READY'),events.index('PICKDROP_COMPLETED'))
                self.assertLess(events.index('NEXT_TARGET_READY'),events.index('PICKDROP_COMPLETED'))
                self.assertLess(events.index('PICKDROP_COMPLETED'),events.index('NEXT_TARGET_LOADED'))
                self.assertLess(events.index('NEXT_TARGET_LOADED'),events.index('NEXT_PICKDROP_SENT'))
                commands=c.robot.client.commands
                self.assertEqual(sum(n=='MOVL' for n,d in commands),0)
                self.assertEqual([d for n,d in commands if n=='START'],['PICKDROP']*2)
                self.assertEqual(c.stack.snapshot()['stacked_count'],2)
            finally:
                c.close()
