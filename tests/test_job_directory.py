import socket
import unittest
from unittest.mock import patch
from robot.protocol import Client,ProtocolError
from robot.job_control import exists
from tests.test_protocol import FakeSocket,GREETING


class JobDirectoryTests(unittest.TestCase):
    def test_moveunti_after_first_twenty_names_in_full_fragmented_directory(self):
        names=[f'JOB_{i:03}' for i in range(120)]
        names[87]='MOVEUNTI'
        body=('\r'.join(','.join(names[i:i+20]) for i in range(0,120,20))+'\r').encode()
        wire=GREETING+b'OK: RJDIR\r\n'+body
        for chunks in ([wire],[wire[i:i+7] for i in range(0,len(wire),7)]):
            with self.subTest(chunk_count=len(chunks)):
                sock=FakeSocket(chunks)
                with patch('socket.create_connection',return_value=sock):
                    self.assertTrue(exists(Client('127.0.0.1')))
                self.assertEqual(sock.sent[-2:],[b'HOSTCTRL_REQUEST RJDIR 2\r\n',b'*\r'])

    def test_missing_job_only_after_complete_directory(self):
        sock=FakeSocket([GREETING,b'OK: RJDIR\r\n',b'HOME,H1ON\r',b'H2ON,H2OFF\r'])
        with patch('socket.create_connection',return_value=sock):
            self.assertFalse(exists(Client('127.0.0.1')))

    def test_timeout_after_first_complete_record_is_not_a_missing_job(self):
        sock=FakeSocket([GREETING,b'OK: RJDIR\r\n',b'HOME,H1ON\r',socket.timeout('remaining list lost')])
        with patch('socket.create_connection',return_value=sock) as create:
            with self.assertRaises(ProtocolError):
                exists(Client('127.0.0.1'))
        create.assert_called_once()

    def test_eof_without_final_record_terminator_is_incomplete(self):
        sock=FakeSocket([GREETING,b'OK: RJDIR\r\n',b'HOME\rMOVEUN'])
        with patch('socket.create_connection',return_value=sock):
            with self.assertRaisesRegex(ProtocolError,'Incomplete directory'):
                exists(Client('127.0.0.1'))

    def test_response_limit_prevents_unbounded_directory(self):
        sock=FakeSocket([GREETING,b'OK: RJDIR\r\n',b'J'*65537])
        with patch('socket.create_connection',return_value=sock):
            with self.assertRaisesRegex(ProtocolError,'receive limit'):
                exists(Client('127.0.0.1'))
