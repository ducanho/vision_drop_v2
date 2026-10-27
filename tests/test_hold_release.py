import socket
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from robot.protocol import Client,OutcomeUnknown
from workflow.cycle import Cycle
from workflow.safety_checks import hold_release_ready
from core.settings import load_config
from tests.test_protocol import FakeSocket,GREETING


class HoldReleaseTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.c=Cycle(load_config(),True,Path(self.tmp.name)/'execution.json')
        self.c.robot.connect()
        self.c.robot.client.holding=True
        self.c.refresh()

    def tearDown(self):
        self.c.close()
        self.tmp.cleanup()

    def test_explicit_release_sends_only_hold_off_and_verifies_readback(self):
        self.assertTrue(self.c.snapshot()['capabilities']['release_hold']['allowed'])
        self.c.release_hold(confirmed=True)
        self.assertFalse(self.c.snapshot()['robot']['flags']['hold'])
        self.assertFalse(self.c.snapshot()['capabilities']['release_hold']['allowed'])
        self.assertEqual([(n,d) for n,d in self.c.robot.client.commands if n in ('HOLD','START','MOVL')],[('HOLD','0')])
        self.assertEqual(self.c.journal.state['state'],'IDLE')

    def test_unconfirmed_or_unacknowledged_release_sends_nothing(self):
        with self.assertRaises(ValueError):
            self.c.release_hold()
        self.c.journal.update(state='FAULT',outcome_unknown=True)
        with self.assertRaises(RuntimeError):
            self.c.release_hold(confirmed=True)
        self.assertFalse(any(n=='HOLD' for n,d in self.c.robot.client.commands))

    def test_external_pendant_running_or_remote_off_blocks_release(self):
        flags=self.c.robot.status()
        for key,value in [('hold_external',True),('hold_pendant',True),('running',True),
                          ('remote',False),('play',False),('teach',True),('hold_command',False),('alarm',True),('error',True)]:
            with self.subTest(key=key),self.assertRaises(RuntimeError):
                hold_release_ready({**flags,key:value})
        self.c.robot.client.running=True
        with self.assertRaises(RuntimeError):
            self.c.release_hold(confirmed=True)
        self.assertFalse(any(n=='HOLD' for n,d in self.c.robot.client.commands))

    def test_lost_release_response_is_fault_without_retry(self):
        original=self.c.robot.client.execute
        def lost(name,data=''):
            result=original(name,data)
            if name=='HOLD':
                raise OutcomeUnknown('HOLD OFF response lost')
            return result
        with patch.object(self.c.robot.client,'execute',lost):
            with self.assertRaises(OutcomeUnknown):
                self.c.release_hold(confirmed=True)
        result=self.c.journal.snapshot()
        self.assertEqual(result['state'],'FAULT')
        self.assertTrue(result['outcome_unknown'])
        self.assertEqual(result['pending']['payload'],'0')
        self.assertEqual(sum(n=='HOLD' for n,d in self.c.robot.client.commands),1)

    def test_wire_framing_for_release_and_lost_reply(self):
        sock=FakeSocket([GREETING,b'OK: HOLD\r\n',b'0000\r\n'])
        with patch('socket.create_connection',return_value=sock):
            Client('127.0.0.1').execute('HOLD','0')
        self.assertEqual(sock.sent[-2:],[b'HOSTCTRL_REQUEST HOLD 2\r\n',b'0\r'])
        sock=FakeSocket([GREETING,b'OK: HOLD\r\n',socket.timeout('lost')])
        with patch('socket.create_connection',return_value=sock) as create:
            with self.assertRaises(OutcomeUnknown):
                Client('127.0.0.1').execute('HOLD','0')
        create.assert_called_once()
