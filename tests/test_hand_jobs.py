"""HAND dispatch/completion tests use a simulator, never the controller."""
import socket
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from core.settings import load_config
from workflow.cycle import Cycle
from robot.protocol import Client, OutcomeUnknown
from tests.test_protocol import FakeSocket, GREETING


class HandJobTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        cfg=load_config()
        cfg['motion']['poll_seconds']=.02
        self.c=Cycle(cfg,True,Path(self.tmp.name)/'execution.json')
        self.c.robot.connect()
        self.c.robot.vacuum_config=cfg['vacuum']
        self.c.robot.vacuum_config['job_timeout']=.3
        self.c.refresh()

    def tearDown(self):
        self.c.close()
        self.tmp.cleanup()

    def finish(self):
        self.c.thread.join(timeout=2)
        self.assertFalse(self.c.busy)
        return self.c.journal.snapshot()

    def test_four_buttons_start_only_their_named_job(self):
        for channel,on,name in [(1,True,'H1ON'),(1,False,'H1OFF'),(2,True,'H2ON'),(2,False,'H2OFF')]:
            self.c.vacuum(channel,on)
            result=self.finish()
            self.assertEqual(result['state'],'IDLE',result['error'])
            self.assertEqual(result['manual_job'],dict(name=name,status='complete'))
            self.assertIsNone(result['pending'])
        commands=self.c.robot.client.commands
        self.assertEqual([d for n,d in commands if n=='START'],['H1ON','H1OFF','H2ON','H2OFF'])
        self.assertFalse(any(n in ('IOWRITE','MOVL') for n,_ in commands))

    def test_ack_is_not_completion_and_second_command_is_blocked(self):
        self.c.vacuum(2,True)
        with self.assertRaises(RuntimeError):
            self.c.vacuum(1,False)
        self.assertTrue(self.c.busy)
        self.assertEqual(self.finish()['manual_job']['status'],'complete')

    def test_lost_start_reply_is_not_retried(self):
        original=self.c.robot.client.execute
        def lost(name,data=''):
            result=original(name,data)
            if name=='START':
                raise OutcomeUnknown('lost HAND response')
            return result
        with patch.object(self.c.robot.client,'execute',lost):
            self.c.vacuum(1,True)
            result=self.finish()
        self.assertEqual(result['state'],'FAULT')
        self.assertTrue(result['outcome_unknown'])
        self.assertEqual(result['pending']['payload'],'H1ON')
        self.assertEqual(sum(n=='START' for n,_ in self.c.robot.client.commands),1)

    def test_hold_cancels_without_resuming(self):
        self.c.vacuum(1,True)
        self.c.hold()
        self.assertEqual(self.finish()['state'],'HOLD')
        self.assertLessEqual(sum(n=='START' for n,_ in self.c.robot.client.commands),1)

    def test_wrong_current_job_is_fault(self):
        with patch.object(self.c.robot,'current_job',return_value='OTHER'):
            self.c.vacuum(1,True)
            result=self.finish()
        self.assertEqual(result['state'],'FAULT')
        self.assertIn('OTHER',result['error'])

    def test_running_after_ack_times_out(self):
        original=self.c.robot.start_hand_job
        def stuck(*args):
            original(*args)
            self.c.robot.client.deadline=time.monotonic()+100
        with patch.object(self.c.robot,'start_hand_job',stuck):
            self.c.vacuum(2,False)
            result=self.finish()
        self.assertEqual(result['state'],'FAULT')
        self.assertIn('timeout',result['error'])

    def test_configured_job_wire_bytes_and_no_implicit_resume(self):
        sock=FakeSocket([GREETING,b'OK: START\r\n',b'0000\r\n'])
        with patch('socket.create_connection',return_value=sock):
            Client('127.0.0.1',allowed_start_jobs=('H2OFF',)).execute('START','H2OFF')
        self.assertEqual(sock.sent[-2:],[b'HOSTCTRL_REQUEST START 6\r\n',b'H2OFF\r'])
        with patch('socket.create_connection') as create:
            for name in ('','H1ON','H2OFF\r\n'):
                with self.assertRaises(ValueError):
                    Client('127.0.0.1',allowed_start_jobs=('H2OFF',)).execute('START',name)
            create.assert_not_called()
