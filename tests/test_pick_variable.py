"""P002 pick handoff; FakeSocket/MockClient only, never real robot sockets."""
import socket
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from core.settings import load_config
from robot.cartesian import Pose
from robot.position_variable import pick_position,verify_p002
from robot.protocol import Client,OutcomeUnknown,encode_command
from tests.test_protocol import FakeSocket,GREETING
from workflow.cycle import Cycle


class PickVariableTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        cfg=load_config()
        cfg['workflow']['repeat_until_empty']=False
        cfg['motion'].update(poll_seconds=.01,timeout=.6,job_timeout=1)
        self.c=Cycle(cfg,True,Path(self.tmp.name)/'execution.json')
        self.c.robot.connect()
        self.c.journal.update(cycle_id='pick-current')
        self.target=dict(x=850.1234,y=100.4567,z=999,rx=1,ry=2,rz=3)

    def tearDown(self):
        self.c.close()
        self.tmp.cleanup()

    def run_pick(self,stage='full_cycle'):
        with patch.object(self.c,'_target',return_value=self.target):
            self.c._run_once(stage)
        return self.c.journal.snapshot()

    def test_p002_uses_yolo_xy_operator_settings_and_live_metadata_not_drop_variables(self):
        self.c.cfg['settings'].update(z_safe=510,rx=179,ry=-2,rz=91)
        self.c.robot.client.tool=3
        self.c.robot.client.posture=5
        self.c.robot.client.p009=['1','0','-999','-999','600','0','0','0','0','0']
        self.c.robot.client.p010=['1','0','-888','-888','330','0','0','0','0','0']
        before=self.c.robot.pose()[0]
        self.c._prepare_pick_variable(self.target)
        self.assertEqual(self.c.robot.client.p002,['1','0','850.123','100.457','510.000','179.000','-2.000','91.000','5','3'])
        self.assertEqual(self.c.robot.pose()[0],before)
        commands=self.c.robot.client.commands
        self.assertFalse(any(n=='MOVL' or (n=='SAVEV' and d in ('4,9','4,10')) for n,d in commands))
        start=self.c._start_moveunti()
        self.assertEqual(start['z'],510)
        commands=self.c.robot.client.commands
        load=next(i for i,(n,d) in enumerate(commands) if n=='LOADV')
        verify=next(i for i,(n,d) in enumerate(commands) if (n,d)==('SAVEV','4,2'))
        job=commands.index(('START','MOVEUNTI'))
        self.assertLess(load,verify)
        self.assertLess(verify,job)
        self.assertEqual(self.c.robot.pose()[0],before)

    def test_wire_p002_loadv_and_savev_and_lost_write_reply(self):
        reference=Pose.parse('1,2,3,180,0,0,5,3,0,0,0,0,0,0')
        fields,payload=pick_position(dict(x=850,y=100,z=500,rx=180,ry=0,rz=90),reference)
        sock=FakeSocket([GREETING,b'OK: LOADV\r\n',b'0000\r\n'])
        with patch('socket.create_connection',return_value=sock):
            Client('127.0.0.1').execute('LOADV',payload)
        self.assertEqual(sock.sent[-2:],[(f'HOSTCTRL_REQUEST LOADV {len(payload)+1}\r\n').encode(),(payload+'\r').encode()])
        sock=FakeSocket([GREETING,b'OK: SAVEV\r\n',(','.join(fields)+'\r').encode()])
        with patch('socket.create_connection',return_value=sock):
            verify_p002(fields,Client('127.0.0.1').execute('SAVEV','4,2'))
        self.assertEqual(sock.sent[-2:],[b'HOSTCTRL_REQUEST SAVEV 4\r\n',b'4,2\r'])
        sock=FakeSocket([GREETING,b'OK: LOADV\r\n',socket.timeout('lost')])
        with patch('socket.create_connection',return_value=sock) as connect:
            with self.assertRaises(OutcomeUnknown):
                Client('127.0.0.1').execute('LOADV',payload)
            connect.assert_called_once()
        for data in ('4,3,1,0,1,2,3,180,0,0,0,0','4,2,1,1,1,2,3,180,0,0,0,0','4,9,1,0,1,2,3,180,0,0,0,0'):
            with self.assertRaises(ValueError):encode_command('LOADV',data)

    def test_readback_checks_every_field_with_one_micron_precision(self):
        fields=('1','0','850','100','500','180','0','90','5','3')
        rounded=list(fields);rounded[2]='850.001'
        verify_p002(fields,','.join(rounded))
        for i in range(10):
            wrong=list(fields);wrong[i]=str(float(wrong[i])+(.002 if 2<=i<8 else 1))
            with self.subTest(field=i),self.assertRaises((RuntimeError,ValueError)):
                verify_p002(fields,','.join(wrong))
        with self.assertRaises(RuntimeError):verify_p002(fields,'1,0,850')

    def test_write_failure_and_mismatch_fault_without_start(self):
        for kind in ('lost','readback'):
            with self.subTest(kind=kind):
                original=self.c.robot.client.execute
                def fail(name,data=''):
                    answer=original(name,data)
                    if kind=='lost' and name=='LOADV':raise OutcomeUnknown('P002 lost')
                    if kind=='readback' and (name,data)==('SAVEV','4,2'):
                        fields=answer.split(',');fields[2]='999';return ','.join(fields)
                    return answer
                self.c.robot.client.commands.clear()
                self.c.journal.update(state='IDLE',pending=None,error='',pick_variable=None)
                self.c.robot.client.holding=False
                with patch.object(self.c.robot.client,'execute',side_effect=fail):result=self.run_pick()
                self.assertEqual(result['state'],'FAULT')
                self.assertFalse(any(n=='START' for n,d in self.c.robot.client.commands))
                self.assertEqual(sum(n=='LOADV' for n,d in self.c.robot.client.commands),1)

    def test_metadata_or_readback_changed_after_verify_blocks_start(self):
        self.c._prepare_pick_variable(self.target)
        self.c.robot.client.tool=1
        with self.assertRaises(RuntimeError):self.c._start_moveunti()
        self.c.robot.client.tool=0
        self.c.robot.client.p002[3]='999'
        with self.assertRaises(RuntimeError):self.c._start_moveunti()
        self.assertFalse(any(n=='START' for n,d in self.c.robot.client.commands))

    def test_new_connection_or_cycle_cannot_reuse_p002_receipt(self):
        self.c._prepare_pick_variable(self.target)
        self.c.robot.connect()
        with self.assertRaises(RuntimeError):self.c._start_moveunti()
        self.c._prepare_pick_variable(self.target)
        self.c.journal.update(cycle_id='other-cycle')
        with self.assertRaises(RuntimeError):self.c._start_moveunti()
        self.assertFalse(any(n=='START' for n,d in self.c.robot.client.commands))


    def test_ready_target_uses_same_write_path_without_resampling(self):
        c=self.c;v=c.vision
        c.journal.update(automatic_repeat=True,target=dict(id=1))
        c.robot.client.last_job='PICKDROP'
        with patch.object(c.robot,'read_b090',side_effect=[0,1]):
            c._arm_drop_prefetch();c._poll_drop_prefetch()
        c.cfg['vision'].update(stable_frames=2,stable_seconds=.02)
        # Re-arm after the fixture config update; production config stays untouched.
        v.prepare_next_pick(after_id=1,cycle_id='pick-current')
        now=time.monotonic();v._prefetch['started_at']=now-.2
        board=dict(id=2,instance=2,xyz=(812.25,104.75,0),confidence=.95)
        v._update_next_pick(1,now-.1,[board]);v._update_next_pick(2,now-.05,[board]);v.connected=True
        c.stack.confirm('pick-current',picked_id=1)
        c.journal.update(cycle_id='next-cycle')
        with patch.object(v,'sample',side_effect=AssertionError('READY must not resample')):
            target=c._target();c._prepare_pick_variable(target);c._start_moveunti()
        self.assertEqual(c.robot.client.p002[2:4],['812.250','104.750'])
        self.assertEqual(v.prefetch_status()['state'],'CONSUMED')
        self.assertFalse(any(n=='MOVL' for n,d in c.robot.client.commands))

    def test_prefetch_change_between_write_and_start_blocks_job(self):
        self.c.vision.connected=True
        self.c._validated_prefetch_identity=self.c.vision.preparation_identity()
        self.c.prefetch['connection_generation']=self.c.robot.connection_generation
        self.c._prepare_pick_variable(self.target)
        self.c.cfg['settings']['z_safe']+=1
        with self.assertRaises(RuntimeError):self.c._start_moveunti()
        self.assertFalse(any(n=='START' for n,d in self.c.robot.client.commands))

    def test_direct_pc_xy_move_is_rejected(self):
        with self.assertRaises(RuntimeError):self.c._move(self.target,'MOVE_XY')
        self.assertFalse(any(n=='MOVL' for n,d in self.c.robot.client.commands))
