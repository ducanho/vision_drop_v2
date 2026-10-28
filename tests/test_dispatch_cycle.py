"""The operator takes over after START MOVEUNTI is accepted."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from core.settings import load_config
from workflow.cycle import Cycle
from workflow.state_machine import Journal
from workflow.safety_checks import real_gate
from robot.protocol import OutcomeUnknown


class DispatchCycleTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.cfg=load_config()
        self.cfg['workflow']=dict(end_after_moveunti_start=True)
        self.cfg['vision'].update(stable_frames=2,stable_seconds=.04)
        self.cfg['motion'].update(poll_seconds=.02,target_timeout=2)
        self.c=Cycle(self.cfg,True,Path(self.tmp.name)/'execution.json')
        self.c.robot.connect()
        self.c.robot.client.scenario='laser_timeout'
        self.c.vision.connect()
        import time
        deadline=time.monotonic()+3
        while not self.c.vision.status()['connected'] and time.monotonic()<deadline:
            time.sleep(.01)

    def tearDown(self):
        self.c.close()
        self.tmp.cleanup()

    def run_cycle(self):
        self.c.start('dispatch-request-001',self.c.journal.state['revision'],'moveunti_only')
        self.c.thread.join(timeout=4)
        self.assertFalse(self.c.busy)
        return self.c.journal.snapshot()

    def test_stops_pc_work_after_ack_without_waiting_for_laser_or_h2on(self):
        result=self.run_cycle()
        self.assertEqual(result['state'],'JOB_DISPATCHED',result['error'])
        self.assertTrue(self.c.robot.status()['running'])
        self.assertFalse(self.c.robot.laser())
        writes=[(n,d) for n,d in self.c.robot.client.commands if n in ('LOADV','MOVL','START','IOWRITE','HOLD')]
        self.assertEqual([n for n,d in writes],['LOADV','START'])
        self.assertEqual(writes[-1],('START','MOVEUNTI'))
        self.assertEqual(list(map(float,writes[0][1].split(',')[6:10])),[500,180,0,90])
        self.assertEqual(result['job_dispatch'],dict(name='MOVEUNTI',accepted=True,completion_verified=False))
        self.assertIsNone(result['pending'])
        self.assertIsNone(result['vacuum_job'])
        self.assertFalse(result['outcome_unknown'])
        with self.assertRaises(RuntimeError):
            self.c.acknowledge()
        with self.assertRaises(RuntimeError):
            self.c.vacuum(2,True)
        # Handoff persists across a server restart without replay or a false FAULT.
        self.assertEqual(Journal(self.c.journal.path).state['state'],'JOB_DISPATCHED')
        self.c.robot.client.running=False
        self.c.robot.client.activity=None
        self.c.acknowledge()
        self.assertEqual(self.c.journal.state['state'],'IDLE')

    def test_lost_start_response_remains_fault_and_is_never_retried(self):
        original=self.c.robot.client.execute
        def lost(name,data=''):
            result=original(name,data)
            if name=='START':
                raise OutcomeUnknown('MOVEUNTI response lost')
            return result
        with patch.object(self.c.robot.client,'execute',lost):
            result=self.run_cycle()
        self.assertEqual(result['state'],'FAULT')
        self.assertTrue(result['outcome_unknown'])
        self.assertIsNone(result['job_dispatch'])
        self.assertEqual(result['pending']['payload'],'MOVEUNTI')
        self.assertEqual(sum(n=='START' for n,d in self.c.robot.client.commands),1)

    def test_real_dispatch_does_not_require_laser_stop_adapter_or_vacuum(self):
        cfg=self.cfg
        # Operator-confirmed handoff does not require static commissioning flags.
        self.assertIsNone(cfg['motion']['expected_tool'])
        self.assertIsNone(cfg['motion']['expected_posture'])
        self.assertFalse(cfg['commissioning']['path_verified'])
        self.assertFalse(cfg['commissioning']['vacuum_only'])
        self.assertFalse(cfg['laser']['verified'])
        self.assertIsNone(cfg['job']['max_descent_mm'])
        for stage in ('xy_only','moveunti_only'):
            real_gate(cfg,stage)
        cfg['job']['name']='OTHER'
        with self.assertRaises(RuntimeError):
            real_gate(cfg,'moveunti_only')
        # full_cycle uses the confirmed PICKDROP contract, not legacy MOVEUNTI.
        real_gate(cfg,'full_cycle')

    def test_real_branch_runs_with_live_metadata_and_without_static_tool_or_path_flags(self):
        self.c.mock=False
        self.c.robot.mock=False
        self.c.robot.client.tool=3
        self.c.robot.client.posture=1
        result=self.run_cycle()
        self.assertEqual(result['state'],'JOB_DISPATCHED',result['error'])
        position=next(d for n,d in self.c.robot.client.commands if n=='LOADV')
        self.assertEqual(position.split(',')[10:12],['1','3'])

    def test_tool_change_after_p002_write_faults_before_start_job(self):
        self.c.mock=False
        self.c.robot.mock=False
        original=self.c.robot.client.execute
        def change_tool(name,data=''):
            result=original(name,data)
            if name=='LOADV' and data.startswith('4,2,'):
                self.c.robot.client.tool=1
            return result
        with patch.object(self.c.robot.client,'execute',change_tool):
            result=self.run_cycle()
        self.assertEqual(result['state'],'FAULT')
        self.assertFalse(any(n=='START' for n,d in self.c.robot.client.commands))

    def test_read_failure_after_accepted_handoff_does_not_send_hold(self):
        original=self.c.refresh
        def read():
            if self.c.journal.state['state']=='JOB_DISPATCHED':
                raise RuntimeError('telemetry unavailable after accepted START')
            return original()
        with patch.object(self.c,'refresh',read):
            result=self.run_cycle()
        self.assertEqual(result['state'],'JOB_DISPATCHED')
        self.assertTrue(result['job_dispatch']['accepted'])
        self.assertFalse(any(n=='HOLD' for n,d in self.c.robot.client.commands))
