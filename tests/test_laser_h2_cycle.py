"""Operator-selected completion: laser ON plus stopped, then H2ON once."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from core.settings import load_config,validate_settings
from workflow.cycle import Cycle


class LaserH2CycleTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        cfg=load_config()
        cfg['workflow']['repeat_until_empty']=False
        cfg['drop'].pop('position_variable',None)
        cfg['drop']['first_drop_z_mm']=330
        cfg['vision'].update(stable_frames=2,stable_seconds=.04)
        # Completion polling now includes the JOB's approach to P002.
        cfg['motion'].update(poll_seconds=.02,target_timeout=2,job_timeout=1)
        self.c=Cycle(cfg,True,Path(self.tmp.name)/'execution.json')
        self.c.connect()
        import time
        deadline=time.monotonic()+3
        while not self.c.vision.status()['connected'] and time.monotonic()<deadline:
            time.sleep(.01)

    def tearDown(self):
        self.c.close()
        self.tmp.cleanup()

    def run_cycle(self):
        self.c.start('laser-h2-request-001',self.c.journal.state['revision'])
        self.c.thread.join(timeout=4)
        self.assertFalse(self.c.busy)
        if self.c.journal.state['state']=='AWAIT_DROP_CONFIRMATION':
            self.c.confirm_drop(self.c.journal.state['cycle_id'],confirmed=True)
        return self.c.journal.snapshot()

    def assert_no_h2on(self):
        self.assertFalse(any(n=='START' and d=='H2ON' for n,d in self.c.robot.client.commands))

    def test_real_branch_eighty_mm_s_and_h2on_only_after_laser_and_stopped(self):
        self.c.cfg['settings']['movl_speed']=80
        self.c.mock=False
        self.c.robot.mock=False
        self.assertTrue(self.c.snapshot()['capabilities']['full_cycle']['allowed'])
        original=self.c.robot.start_hand_job
        def verify(channel,on,config=None):
            if not on:
                return original(channel,on,config)
            self.assertEqual((channel,on),(2,True))
            self.assertFalse(self.c.robot.status()['running'])
            self.assertTrue(self.c.robot.laser())
            return original(channel,on,config)
        with patch.object(self.c.robot,'start_hand_job',verify):
            result=self.run_cycle()
        self.assertEqual(result['state'],'COMPLETE',result['error'])
        self.assertEqual(result['moveunti_completion'],dict(criteria='laser_on_and_stopped',laser=True,running=False))
        commands=self.c.robot.client.commands
        self.assertEqual([d for n,d in commands if n=='START'],['MOVEUNTI','H2ON','DROP','H2OFF'])
        self.assertEqual(next(d for n,d in commands if n=='MOVL').split(',')[1],'80')
        self.assertEqual(result['vacuum_job'],dict(name='H2ON',status='complete'))
        self.assertEqual(self.c.robot.pose()[0]['z'],530)
        self.assertFalse(any(n=='IOWRITE' for n,d in commands))
        validate_settings({'movl_speed':500})
        with self.assertRaises(ValueError):
            validate_settings({'movl_speed':500.1})

    def test_stopped_with_laser_off_never_starts_h2on(self):
        self.c.robot.client.scenario='unproven_stop'
        result=self.run_cycle()
        self.assertEqual(result['state'],'FAULT')
        self.assertIn('timeout',result['error'])
        self.assert_no_h2on()

    def test_laser_on_while_running_never_starts_h2on(self):
        client=self.c.robot.client
        client.scenario='laser_timeout'
        original=client.execute
        def laser_early(name,data=''):
            result=original(name,data)
            if name=='IOREAD' and data.startswith('20050,') and client.activity=='job':
                return '1'
            return result
        with patch.object(client,'execute',laser_early):
            result=self.run_cycle()
        self.assertEqual(result['state'],'FAULT')
        self.assertIn('timeout',result['error'])
        self.assert_no_h2on()

    def test_laser_unknown_during_job_never_starts_h2on(self):
        client=self.c.robot.client
        original=client.execute
        def unreadable(name,data=''):
            if name=='IOREAD' and data.startswith('20050,') and client.activity=='job':
                raise RuntimeError('IOREAD unavailable')
            return original(name,data)
        with patch.object(client,'execute',unreadable):
            result=self.run_cycle()
        self.assertEqual(result['state'],'FAULT')
        self.assertIn('UNKNOWN',result['error'])
        self.assert_no_h2on()

    def test_laser_drops_before_h2on_command_is_blocked(self):
        original=self.c._execute_hand_job
        def drop(*args,**kwargs):
            self.c.robot.client.laser=False
            return original(*args,**kwargs)
        with patch.object(self.c,'_execute_hand_job',drop):
            result=self.run_cycle()
        self.assertEqual(result['state'],'FAULT')
        self.assert_no_h2on()

    def test_short_job_can_finish_between_status_polls(self):
        original=self.c._start_moveunti
        def short():
            result=original()
            self.c.robot.client.deadline=0
            return result
        with patch.object(self.c,'_start_moveunti',short):
            result=self.run_cycle()
        self.assertEqual(result['state'],'COMPLETE',result['error'])
        self.assertEqual([d for n,d in self.c.robot.client.commands if n=='START'],['MOVEUNTI','H2ON','DROP','H2OFF'])
