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
        for command,data in [('SAVEV','4,11'),('LOADV','4,11,1,0,1,2,3,180,0,0,0,0')]:
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


class DropVariableFlowTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        cfg=load_config()
        cfg['workflow']['repeat_until_empty']=False
        cfg['motion'].update(poll_seconds=.01,settle_samples=2)
        self.c=Cycle(cfg,True,Path(self.tmp.name)/'execution.json')
        self.c.robot.connect()

    def tearDown(self):
        self.c.close()
        self.tmp.cleanup()

    def run_drop(self,cid='variable-cycle-001'):
        self.c.journal.update(state='SYSTEM_CHECK',cycle_id=cid,drop_attempt_cycle_id=None,drop_variable=None)
        def picked():
            self.c.robot.client.laser=True
            self.c.robot.client.last_job='MOVEUNTI'
        target=dict(x=850,y=100,z=500,rx=180,ry=0,rz=90)
        with patch.object(self.c,'_target',return_value=target),patch.object(self.c,'_job',side_effect=picked):
            self.c._run('full_cycle')
        return self.c.journal.snapshot()

    def test_write_and_verify_before_drop_no_pc_descent_first_three_boards(self):
        for i,z in enumerate((330,342,354)):
            s=self.run_drop(f'variable-cycle-{i}')
            self.assertEqual(s['state'],'COMPLETE',s['error'])
            self.assertTrue(s['drop_variable']['verified'])
            self.assertEqual(float(self.c.robot.client.p010[4]),z)
            self.assertEqual(self.c.robot.pose()[0]['z'],z+200)
        commands=self.c.robot.client.commands
        self.assertEqual(sum(n=='LOADV' and d.startswith('4,10,') for n,d in commands),3)
        self.assertEqual(sum(n=='MOVL' for n,d in commands),3)
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],3)
        first_drop=commands.index(('START','DROP'))
        self.assertEqual(commands[first_drop-1][0],'RSTATS')
        self.assertGreaterEqual(sum(n=='SAVEV' for n,d in commands[:first_drop]),2)
        self.assertTrue(any(n=='LOADV' for n,d in commands[:first_drop]))

    def test_wrong_readback_blocks_drop_h2off_and_count(self):
        execute=self.c.robot.client.execute
        def mismatch(name,data=''):
            answer=execute(name,data)
            if name=='SAVEV' and data=='4,10' and self.c.robot.client.drop_uses_p010:
                fields=answer.split(',')
                fields[4]='999'
                return ','.join(fields)
            return answer
        with patch.object(self.c.robot.client,'execute',side_effect=mismatch):
            s=self.run_drop()
        self.assertEqual(s['state'],'FAULT')
        self.assertFalse(any(n=='START' and d in ('DROP','H2OFF') for n,d in self.c.robot.client.commands))
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],0)

    def test_drop_must_stop_at_p010_before_h2off(self):
        execute=self.c.robot.client.execute
        self.c.cfg['drop']['job_timeout']=.3
        def old_drop(name,data=''):
            answer=execute(name,data)
            if name=='START' and data=='DROP':
                self.c.robot.client.drop_uses_p010=False
            return answer
        with patch.object(self.c.robot.client,'execute',side_effect=old_drop):
            s=self.run_drop()
        self.assertEqual(s['state'],'FAULT')
        self.assertIn('P010',s['error'])
        self.assertFalse(any(n=='START' and d=='H2OFF' for n,d in self.c.robot.client.commands))
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],0)

    def test_p010_uses_p009_safe_xy_and_posture_instead_of_old_p010_or_pick(self):
        self.c.robot.client.p009=['1','0','650.123','-750.456','600','179.99','0.12','45.67','0','0']
        safe=self.c.robot.client.p009[:]
        self.c.robot.client.p010=['1','0','900','900','450','180','0','90','0','0']
        s=self.run_drop()
        self.assertEqual(s['state'],'COMPLETE',s['error'])
        result=self.c.robot.client.p010
        self.assertEqual(result[:4],safe[:4])
        self.assertEqual(result[5:],safe[5:])
        self.assertEqual(float(result[4]),330)
        self.assertEqual(self.c.robot.client.p009,safe)
        self.assertEqual(s['drop_variable']['source'],'P009')
