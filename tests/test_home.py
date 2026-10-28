"""HOME handoff uses the existing JOB, with no PC move or automatic replay."""
import socket
import time
from unittest.mock import patch

from robot.protocol import Client, OutcomeUnknown
from tests.pickdrop_support import PickDropCase
from tests.test_protocol import FakeSocket, GREETING
from workflow.state_machine import Journal


class HomeTests(PickDropCase):
    def test_home_dispatches_once_without_camera_or_writing_positions_or_speed(self):
        self.assertFalse(self.c.vision.connected)
        self.c.home()
        state=self.c.journal.snapshot()
        self.assertEqual(self.starts(),['HOME'])
        self.assertEqual(state['state'],'JOB_DISPATCHED')
        self.assertEqual(state['job_dispatch'],dict(name='HOME',accepted=True,completion_verified=False))
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],0)
        self.assertFalse(any(n in ('MOVL','LOADV','IOWRITE','SVON') for n,d in self.c.robot.client.commands))
        with self.assertRaises(RuntimeError):
            self.c.home()
        with self.assertRaises(RuntimeError):
            self.c.start('home-pick-request',state['revision'])
        with self.assertRaises(RuntimeError):
            self.c.acknowledge()
        self.c.robot.client.deadline=0
        self.c.acknowledge()
        self.assertEqual(self.c.journal.state['state'],'IDLE')

    def test_complete_is_allowed_but_busy_fault_hold_pending_unknown_are_blocked(self):
        for state in ('FAULT','HOLD','PICKDROP_RUNNING','START_HOME','SERVO_ENABLING'):
            self.c.journal.update(state=state)
            with self.assertRaises(RuntimeError):
                self.c.home()
        self.c.journal.update(state='IDLE',outcome_unknown=True)
        with self.assertRaises(RuntimeError):
            self.c.home()
        self.c.journal.update(outcome_unknown=False,pending=dict(name='START'))
        with self.assertRaises(RuntimeError):
            self.c.home()
        self.assertEqual(self.starts(),[])
        self.c.journal.update(state='COMPLETE',pending=None)
        self.c.home()
        self.assertEqual(self.starts(),['HOME'])

    def test_live_robot_readiness_is_required(self):
        for attribute in ('holding','alarm','running'):
            setattr(self.c.robot.client,attribute,True)
            with self.assertRaises(RuntimeError):
                self.c.home()
            setattr(self.c.robot.client,attribute,False)
        self.c.robot.client.servo=False
        with self.assertRaises(RuntimeError):
            self.c.home()
        self.assertEqual(self.starts(),[])

    def test_missing_home_no_start(self):
        self.c.robot.client.available_home=False
        with self.assertRaisesRegex(RuntimeError,'HOME'):
            self.c.home()
        self.assertEqual(self.starts(),[])

    def test_lost_home_reply_is_unknown_and_never_retried(self):
        self.c.robot.client.scenario='lost_home_reply'
        with self.assertRaises(OutcomeUnknown):
            self.c.home()
        self.assertEqual(self.starts(),['HOME'])
        self.assertEqual(self.c.journal.state['state'],'FAULT')
        self.assertTrue(self.c.journal.state['outcome_unknown'])
        self.assertEqual(self.c.journal.state['pending']['payload'],'HOME')
        with self.assertRaises(RuntimeError):
            self.c.home()
        self.assertTrue(Journal(self.path).state['outcome_unknown'])

    def test_restart_between_home_intent_and_ack_does_not_replay(self):
        self.c.journal.update(state='START_HOME')
        self.c.journal.intent('START','HOME')
        restored=Journal(self.path)
        self.assertEqual(restored.state['state'],'FAULT')
        self.assertTrue(restored.state['outcome_unknown'])
        self.assertEqual(self.starts(),[])

    def test_home_wire_and_no_retry(self):
        sock=FakeSocket([GREETING,b'OK: START\r\n',socket.timeout('lost')])
        with patch('socket.create_connection',return_value=sock) as connect:
            with self.assertRaises(OutcomeUnknown):
                Client('127.0.0.1',allowed_start_jobs=('HOME',)).execute('START','HOME')
        connect.assert_called_once()
        self.assertEqual(sock.sent[-2:],[b'HOSTCTRL_REQUEST START 5\r\n',b'HOME\r'])
