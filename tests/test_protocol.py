"""Adapted wire-framing cases from workspace protocol tests."""
import socket
import unittest
from unittest.mock import patch
from robot.protocol import Client, ProtocolError, OutcomeUnknown, encode_command
from robot.cartesian import Pose, build_move
from robot.status import Status

GREETING=b'OK: NX Information Server (3.00).\r\n'


class FakeSocket:
    def __init__(self,chunks):
        self.chunks=list(chunks)
        self.sent=[]
    def __enter__(self):
        return self
    def __exit__(self,*args):
        pass
    def settimeout(self,value):
        pass
    def sendall(self,wire):
        self.sent.append(wire)
    def recv(self,count):
        if not self.chunks:
            return b''
        value=self.chunks.pop(0)
        if isinstance(value,Exception):
            raise value
        return value


class ProtocolTests(unittest.TestCase):
    def test_fragmented_and_coalesced_status(self):
        wire=GREETING+b'OK: RSTATS\r\n194,64\r'
        for chunks in ([wire],[wire[i:i+1] for i in range(len(wire))]):
            sock=FakeSocket(chunks)
            with patch('socket.create_connection',return_value=sock):
                self.assertEqual(Client('127.0.0.1').execute('RSTATS'),'194,64')
            self.assertEqual(sock.sent,[b'CONNECT Robot_access\r\n',b'HOSTCTRL_REQUEST RSTATS 0\r\n'])

    def test_start_explicit_job_and_exact_size(self):
        sock=FakeSocket([GREETING,b'OK: START\r\n',b'0000\r\n'])
        with patch('socket.create_connection',return_value=sock):
            self.assertEqual(Client('127.0.0.1').execute('START','MOVEUNTI'),'0000')
        self.assertEqual(sock.sent[-2:],[b'HOSTCTRL_REQUEST START 9\r\n',b'MOVEUNTI\r'])

    def test_lost_start_reply_never_retries(self):
        sock=FakeSocket([GREETING,b'OK: START\r\n',socket.timeout('lost')])
        with patch('socket.create_connection',return_value=sock) as create:
            with self.assertRaises(OutcomeUnknown):
                Client('127.0.0.1').execute('START','MOVEUNTI')
        create.assert_called_once()

    def test_no_payload_before_ack(self):
        sock=FakeSocket([GREETING,b'NG: rejected\r\n'])
        with patch('socket.create_connection',return_value=sock):
            with self.assertRaises(ProtocolError):
                Client('127.0.0.1').execute('START','MOVEUNTI')
        self.assertNotIn(b'MOVEUNTI\r',sock.sent)

    def test_invalid_command_does_not_open_socket(self):
        with patch('socket.create_connection') as create:
            for name,data in [('START',''),('START','OTHER'),('START','MOVEUNTI\r\n'),('SVON','2'),
                              ('RESET',''),('HOLD','2'),('IOWRITE','20050,8,1'),('IOREAD','20059,8')]:
                with self.subTest(name=name,data=data),self.assertRaises(ValueError):
                    Client('127.0.0.1').execute(name,data)
            create.assert_not_called()

    def test_movl_preserves_metadata(self):
        p=Pose.parse('1,2,3,4,5,6,1,2,10,20,30,40,50,60')
        data=build_move('MOVL','5',['850','100','500','180','0','0'],p)
        self.assertEqual(len(data.split(',')),17)
        self.assertEqual(data.split(',')[9:],list(p.fields[6:]))

    def test_every_status_bit(self):
        for bit in range(8):
            s=Status(1<<bit,0)
            methods=['is_step','is_one_cycle','is_auto','is_running','is_safety_speed_operation','is_teach','is_play','is_command_remote']
            for index,method in enumerate(methods):
                self.assertEqual(getattr(s,method)(),index==bit)
        for bit in range(1,7):
            s=Status(0,1<<bit)
            methods=['is_programming_pendant_hold','is_external_hold','is_command_hold','is_alarm_occurring','is_error_occurring','is_servo_on']
            for index,method in enumerate(methods,1):
                self.assertEqual(getattr(s,method)(),index==bit)
