import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch,PropertyMock
from workflow.cycle import Cycle
from workflow.drop import StackStore
from workflow.state_machine import Journal
from core.settings import load_config
from robot.protocol import OutcomeUnknown


class DropTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.path=Path(self.tmp.name)/'execution.json'
        self.cfg=load_config()
        self.cfg['workflow']['repeat_until_empty']=False
        self.cfg['drop'].pop('position_variable',None)
        self.cfg['drop']['first_drop_z_mm']=330
        self.cfg['motion'].update(poll_seconds=.01,settle_samples=2,timeout=.7)
        self.cfg['drop']['job_timeout']=.7
        self.c=Cycle(self.cfg,True,self.path)
        self.c.robot.connect()

    def tearDown(self):
        self.c.close()
        self.tmp.cleanup()

    def run_from_pick_completed(self,cycle_id='drop-001'):
        # Only replace completed pick inputs; exercise unchanged H2ON and DROP START.
        self.c.journal.update(state='SYSTEM_CHECK',cycle_id=cycle_id,target=None,pending=None,
                              outcome_unknown=False,error='',drop_release=None,drop_target=None)
        self.c.cancel.clear()
        target={**self.c.robot.pose()[0],**{k:self.cfg['settings'][k] for k in ('rx','ry','rz')},'z':500}
        def picked():
            self.c.robot.client.last_job='MOVEUNTI'
            self.c.robot.client.laser=True
            self.c.journal.update(job_start_attempted=True)
        with patch.object(self.c,'_target',return_value=target),patch.object(self.c,'_job',side_effect=picked):
            self.c._run('full_cycle')
        return self.c.journal.snapshot()

    def test_first_second_third_heights_and_automatic_count(self):
        for number,z in enumerate((330,342,354),1):
            result=self.run_from_pick_completed(f'drop-{number}')
            self.assertEqual(result['state'],'COMPLETE',result['error'])
            self.assertEqual(result['drop_target']['z'],z)
            self.assertEqual(self.c.stack.snapshot()['stacked_count'],number)
            self.assertEqual(result['drop_safe_pose']['z'],600)
            for axis in ('x','y','rx','ry','rz'):
                self.assertEqual(result['drop_target'][axis],result['drop_safe_pose'][axis])
            self.c.confirm_drop(f'drop-{number}',confirmed=True)
            self.assertEqual(self.c.journal.state['state'],'COMPLETE')
            self.assertEqual(self.c.stack.snapshot()['stacked_count'],number)
        self.assertEqual(self.c.stack.snapshot()['next_drop_z_mm'],366)

    def test_h2off_is_real_start_after_drop_z_is_reached_and_stopped(self):
        original=self.c.robot.start_hand_job
        seen=[]
        def verify(channel,on,config=None):
            if channel==2 and on is False:
                seen.append(self.c.robot.pose()[0])
                self.assertFalse(self.c.robot.status()['running'])
                self.assertEqual(seen[-1]['z'],330)
                self.assertEqual(self.c.journal.state['state'],'EXECUTING_H2OFF')
            return original(channel,on,config)
        with patch.object(self.c.robot,'start_hand_job',verify):
            result=self.run_from_pick_completed()
        self.assertEqual(result['state'],'COMPLETE',result['error'])
        jobs=[d for n,d in self.c.robot.client.commands if n=='START']
        self.assertEqual(jobs,['H2ON','DROP','H2OFF'])
        self.assertEqual(len(seen),1)
        self.assertTrue(result['drop_release']['job_completed'])
        self.assertFalse(result['drop_release']['board_release_confirmed'])

    def test_lift_is_z_plus_200_after_h2off_and_count_is_saved_first(self):
        original=self.c._move
        lifts=[]
        def verify(target,state,**kwargs):
            if state=='POST_H2OFF_MOVE':
                before=self.c.robot.pose()[0]
                self.assertEqual(self.c.stack.snapshot()['stacked_count'],1)
                self.assertTrue(self.c.journal.state['drop_release']['job_completed'])
                self.assertEqual(target['z'],before['z']+200)
                for axis in ('x','y','rx','ry','rz'):
                    self.assertEqual(target[axis],before[axis])
                lifts.append(target)
            return original(target,state,**kwargs)
        with patch.object(self.c,'_move',side_effect=verify):
            result=self.run_from_pick_completed()
        self.assertEqual(result['state'],'COMPLETE',result['error'])
        self.assertEqual(len(lifts),1)
        self.assertEqual(self.c.robot.pose()[0]['z'],530)
        self.assertEqual(result['drop_release']['count_basis'],'h2off_completed')

    def test_lift_failure_does_not_undo_or_duplicate_completed_h2off_count(self):
        original=self.c._move
        def fail(target,state,**kwargs):
            if state=='POST_H2OFF_MOVE':
                raise RuntimeError('lift failed')
            return original(target,state,**kwargs)
        with patch.object(self.c,'_move',side_effect=fail):
            result=self.run_from_pick_completed()
        self.assertEqual(result['state'],'FAULT')
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],1)
        self.assertFalse(self.c.stack.confirm('drop-001'))
        self.assertEqual(sum(n=='START' and d=='H2OFF' for n,d in self.c.robot.client.commands),1)

    def test_position_not_reached_blocks_h2off(self):
        original=self.c._move
        def no_descent(target,state,**kwargs):
            if state!='MOVING_TO_DROP_Z':
                return original(target,state,**kwargs)
        with patch.object(self.c,'_move',side_effect=no_descent):
            result=self.run_from_pick_completed()
        self.assertEqual(result['state'],'FAULT')
        self.assertFalse(any(n=='START' and d=='H2OFF' for n,d in self.c.robot.client.commands))
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],0)

    def test_h2off_lost_response_faults_and_never_retries_or_counts(self):
        original=self.c.robot.client.execute
        def lost(name,data=''):
            result=original(name,data)
            if name=='START' and data=='H2OFF':
                raise OutcomeUnknown('H2OFF response lost')
            return result
        with patch.object(self.c.robot.client,'execute',side_effect=lost):
            result=self.run_from_pick_completed()
        self.assertEqual(result['state'],'FAULT')
        self.assertEqual(result['pending']['payload'],'H2OFF')
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],0)
        self.assertEqual(sum(n=='START' and d=='H2OFF' for n,d in self.c.robot.client.commands),1)
        with self.assertRaises(RuntimeError):
            self.c.confirm_drop('drop-001',confirmed=True)

    def test_h2off_timeout_does_not_count(self):
        original=self.c.robot.start_hand_job
        def stuck(channel,on,config=None):
            original(channel,on,config)
            if not on:
                config['job_timeout']=.05
                self.c.robot.client.deadline=time.monotonic()+100
        with patch.object(self.c.robot,'start_hand_job',side_effect=stuck):
            result=self.run_from_pick_completed()
        self.assertEqual(result['state'],'FAULT')
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],0)
        self.assertIsNone(result.get('drop_release'))
        self.assertEqual(sum(n=='START' and d=='H2OFF' for n,d in self.c.robot.client.commands),1)

    def test_unverified_h2off_completion_never_counts(self):
        original=self.c.robot.current_job
        def wrong():
            return 'OTHER' if self.c.robot.client.last_job=='H2OFF' else original()
        with patch.object(self.c.robot,'current_job',side_effect=wrong):
            result=self.run_from_pick_completed()
        self.assertEqual(result['state'],'FAULT')
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],0)
        self.assertIsNone(result.get('drop_release'))

    def test_same_drop_cycle_cannot_send_h2off_twice(self):
        self.run_from_pick_completed()
        writes=[v for v in self.c.robot.client.commands if v[0] in ('MOVL','START','HOLD')]
        with self.assertRaises(RuntimeError):
            self.c._finish_drop()
        self.assertEqual([v for v in self.c.robot.client.commands if v[0] in ('MOVL','START','HOLD')],writes)

    def test_wrong_drop_safe_z_blocks_descent_and_h2off(self):
        self.c.robot.client.drop_pose[2]=329
        result=self.run_from_pick_completed()
        self.assertEqual(result['state'],'FAULT')
        self.assertNotIn('MOVING_TO_DROP_Z',[v['state'] for v in result['history']])
        self.assertFalse(any(n=='START' and d=='H2OFF' for n,d in self.c.robot.client.commands))

    def test_height_above_safe_is_rejected(self):
        self.c.stack.data['stacked_count']=24
        result=self.run_from_pick_completed()
        self.assertEqual(result['state'],'FAULT')
        self.assertNotIn('MOVING_TO_DROP_Z',[v['state'] for v in result['history']])
        self.assertFalse(any(n=='START' and d=='H2OFF' for n,d in self.c.robot.client.commands))

    def test_duplicate_confirmation_and_reset_preserve_thickness_and_do_not_send_commands(self):
        self.c.stack.configure(15)
        self.run_from_pick_completed()
        self.c.confirm_drop('drop-001',confirmed=True)
        self.c.confirm_drop('drop-001',confirmed=True)
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],1)
        writes=lambda:[(n,d) for n,d in self.c.robot.client.commands if n in ('MOVL','START','HOLD','IOWRITE')]
        commands=writes()
        with self.assertRaises(ValueError):
            self.c.reset_stack()
        self.c.reset_stack(confirmed=True)
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],0)
        self.assertEqual(self.c.stack.snapshot()['next_drop_z_mm'],330)
        self.assertEqual(self.c.stack.snapshot()['board_thickness_mm'],15)
        self.assertEqual(writes(),commands)
        self.assertFalse(self.c.stack.confirm('drop-001'))

    def test_reset_blocked_when_busy_or_in_drop_phase(self):
        with patch.object(Cycle,'busy',new_callable=PropertyMock,return_value=True):
            with self.assertRaises(RuntimeError):
                self.c.reset_stack(confirmed=True)
        self.c.journal.update(state='EXECUTING_H2OFF')
        with self.assertRaises(RuntimeError):
            self.c.reset_stack(confirmed=True)

    def test_restart_keeps_count_and_never_replays_jobs(self):
        self.run_from_pick_completed()
        self.c.confirm_drop('drop-001',confirmed=True)
        self.c.close()
        self.c=Cycle(self.cfg,True,self.path)
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],1)
        self.assertEqual(self.c.stack.snapshot()['last_completed_drop_cycle_id'],'drop-001')
        self.assertEqual(self.c.robot.client.commands,[])
        self.assertEqual(self.c.stack.snapshot()['next_drop_z_mm'],342)

    def test_restart_after_count_write_failure_never_recounts_or_replays(self):
        with patch.object(self.c.stack,'confirm',side_effect=OSError('disk full')):
            result=self.run_from_pick_completed()
        self.assertEqual(result['state'],'FAULT')
        self.assertTrue(result['drop_release']['job_completed'])
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],0)
        self.c.close()
        self.c=Cycle(self.cfg,True,self.path)
        self.assertEqual(self.c.journal.state['state'],'FAULT')
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],0)
        self.assertEqual(self.c.robot.client.commands,[])

    def test_restart_with_h2off_intent_never_counts_or_replays(self):
        self.c.journal.update(state='EXECUTING_H2OFF',cycle_id='unknown-drop')
        self.c.journal.intent('START','H2OFF')
        self.c.close()
        self.c=Cycle(self.cfg,True,self.path)
        self.assertEqual(self.c.journal.state['state'],'FAULT')
        self.assertEqual(self.c.journal.state['pending']['payload'],'H2OFF')
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],0)
        self.assertEqual(self.c.robot.client.commands,[])

    def test_stack_commit_is_atomic_and_idempotent_after_restart(self):
        store=self.c.stack
        with patch.object(store,'_save',side_effect=OSError('disk full')):
            with self.assertRaises(OSError):
                store.confirm('cycle-atomic')
        self.assertEqual(store.snapshot()['stacked_count'],0)
        store.confirm('cycle-atomic')
        restored=StackStore(store.path,self.cfg['drop'])
        self.assertFalse(restored.confirm('cycle-atomic'))
        self.assertEqual(restored.snapshot()['stacked_count'],1)
