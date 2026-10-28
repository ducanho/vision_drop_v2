import socket
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from robot.position_variable import parse_p010,with_z,verify_p010
from robot.protocol import Client,OutcomeUnknown,encode_command
from tests.test_protocol import FakeSocket,GREETING
from core.settings import load_config
from workflow.cycle import Cycle


class PositionVariableTests(unittest.TestCase):
    def test_only_z_changes_and_other_fields_are_preserved(self):
        original='1,0,701.123,-805.456,600.000,179.99,-0.12,89.90,5,3'
        fields=parse_p010(original)
        expected,payload,pose=with_z(fields,330)
        self.assertEqual(expected[:4],fields[:4])
        self.assertEqual(expected[5:],fields[5:])
        self.assertEqual(expected[4],'330.000')
        self.assertEqual(pose['z'],330)
        verify_p010(expected,','.join(expected))
        self.assertEqual(payload,'4,10,1,0,701.123,-805.456,330.000,179.99,-0.12,89.90,5,3')

    def test_invalid_variable_and_coordinate_data_rejected(self):
        for raw in ['0,1,2,3,4,5,6,0','1,1,1,2,3,180,0,0,0,0','1,0,NaN,2,3,180,0,0,0,0']:
            with self.assertRaises(RuntimeError):
                parse_p010(raw)
        for command,data in [('SAVEV','4,12'),('LOADV','4,11,1,0,1,2,3,180,0,0,0,0')]:
            with self.assertRaises(ValueError):
                encode_command(command,data)

    def test_loadv_lost_reply_is_unknown_and_not_retried(self):
        payload='4,10,1,0,700,-800,330,180,0,0,0,0'
        sock=FakeSocket([GREETING,b'OK: LOADV\r\n',socket.timeout('lost')])
        with patch('socket.create_connection',return_value=sock) as connect:
            with self.assertRaises(OutcomeUnknown):
                Client('127.0.0.1').execute('LOADV',payload)
        connect.assert_called_once()
        self.assertEqual(sock.sent[-1],(payload+'\r').encode())
