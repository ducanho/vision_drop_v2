"""Durable stack and operator compatibility after PICKDROP completion."""
from unittest.mock import patch, PropertyMock
from tests.pickdrop_support import PickDropCase
from workflow.drop import StackStore
from workflow.cycle import Cycle


class DropTests(PickDropCase):
    def test_decimal_heights_and_p009_metadata_preserved_over_three_cycles(self):
        self.c.robot.client.p009=['1','0','650.123','-750.456','600','179.99','0.12','45.67','5','3']
        before=self.c.robot.client.p009[:]
        self.c.stack.configure_values(dict(first_drop_z_mm=330.123,board_thickness_mm=12.345))
        for i,z in enumerate((330.123,342.468,354.813)):
            result=self.run_cycle(f'height-{i}')
            self.assertEqual(result['state'],'COMPLETE',result['error'])
            fields=self.c.robot.client.p010
            self.assertEqual(fields[:4],before[:4])
            self.assertEqual(fields[5:],before[5:])
            self.assertEqual(float(fields[4]),z)
            self.assertEqual(self.c.robot.client.p009,before)
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],3)
        self.assertEqual(self.starts(),['PICKDROP']*3)

    def test_height_above_safe_blocks_all_start(self):
        self.c.stack.set_count(24)
        result=self.run_cycle()
        self.assertEqual(result['state'],'FAULT')
        self.assertEqual(self.starts(),[])
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],24)

    def test_duplicate_confirmation_and_reset_preserve_thickness_and_do_not_send_commands(self):
        self.c.stack.configure(15)
        self.run_cycle()
        self.c.confirm_drop('pickdrop-001',confirmed=True)
        self.c.confirm_drop('pickdrop-001',confirmed=True)
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
        self.assertFalse(self.c.stack.confirm('pickdrop-001'))

    def test_reset_blocked_when_busy_or_in_drop_phase(self):
        with patch.object(Cycle,'busy',new_callable=PropertyMock,return_value=True):
            with self.assertRaises(RuntimeError):
                self.c.reset_stack(confirmed=True)
        self.c.journal.update(state='PICKDROP_RUNNING')
        with self.assertRaises(RuntimeError):
            self.c.reset_stack(confirmed=True)

    def test_restart_keeps_count_and_never_replays_jobs(self):
        self.run_cycle()
        self.c.confirm_drop('pickdrop-001',confirmed=True)
        self.c.close()
        self.c=Cycle(self.cfg,True,self.path)
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],1)
        self.assertEqual(self.c.stack.snapshot()['last_completed_drop_cycle_id'],'pickdrop-001')
        self.assertEqual(self.c.robot.client.commands,[])
        self.assertEqual(self.c.stack.snapshot()['next_drop_z_mm'],342)

    def test_restart_after_count_write_failure_never_recounts_or_replays(self):
        with patch.object(self.c.stack,'confirm',side_effect=OSError('disk full')):
            result=self.run_cycle()
        self.assertEqual(result['state'],'FAULT')
        self.assertTrue(result['pickdrop']['completed'])
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],0)
        self.c.close()
        self.c=Cycle(self.cfg,True,self.path)
        self.assertEqual(self.c.journal.state['state'],'FAULT')
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],0)
        self.assertEqual(self.c.robot.client.commands,[])

    def test_restart_with_h2off_intent_never_counts_or_replays(self):
        self.c.journal.update(state='PICKDROP_RUNNING',cycle_id='unknown-drop')
        self.c.journal.intent('START','PICKDROP')
        self.c.close()
        self.c=Cycle(self.cfg,True,self.path)
        self.assertEqual(self.c.journal.state['state'],'FAULT')
        self.assertEqual(self.c.journal.state['pending']['payload'],'PICKDROP')
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
