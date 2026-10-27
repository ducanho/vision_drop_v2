import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from core.settings import load_config
from vision.stability import StableTarget
from workflow.cycle import Cycle


def board(board_id=2,x=800,instance=2,uncertain=False):
    return dict(id=board_id,instance=instance,xyz=(x,0,0),confidence=.95,uncertain=uncertain)


class PreparedPickTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        cfg=load_config()
        cfg['vision'].update(stable_frames=5,stable_seconds=1)
        cfg['motion'].update(target_timeout=.08,poll_seconds=.005)
        self.c=Cycle(cfg,True,Path(self.tmp.name)/'execution.json')
        self.c.robot.connect()
        self.c.stack.confirm('previous-board',picked_id=1)
        self.c.journal.update(state='SYSTEM_CHECK',automatic_repeat=True)

    def tearDown(self):
        self.c.close()
        self.tmp.cleanup()

    def prepared(self):
        stable=StableTarget(self.c.cfg['vision'],after_id=1)
        now=time.monotonic()
        for i in range(6):
            stamp=now-1.2+i*.2
            stable.update(i+1,stamp,[board()],stamp)
        return stable

    def test_prepared_history_accepts_one_fresh_frame_and_uses_fresh_xy(self):
        def fresh():
            # Let a distinct post-lift frame arrive past Windows clock granularity.
            time.sleep(.02)
            return 100,time.monotonic(),[board(1,700,10),board(x=801)]
        with patch.object(self.c.vision,'take_next_pick',return_value=self.prepared()),\
             patch.object(self.c.vision,'sample',side_effect=fresh),\
             patch.object(self.c,'_wait',side_effect=AssertionError('Unnecessary dwell')):
            target=self.c._target()
        self.assertEqual(target['x'],801)
        self.assertEqual(self.c.journal.state['target']['id'],2)
        self.assertEqual(self.c.journal.state['target']['frame_id'],100)
        self.assertEqual(self.c.journal.state['target_prefilled_frames'],6)
        self.assertFalse(any(n in ('MOVL','START') for n,d in self.c.robot.client.commands))

    def test_occluded_uncertain_moved_or_new_instance_must_stabilize_again(self):
        for items in ([],[board(uncertain=True)],[board(x=820)],[board(instance=99)],[board(3,800,3)]):
            with self.subTest(items=items):
                self.c.journal.update(target=None)
                frame=100
                def fresh():
                    nonlocal frame
                    frame+=1
                    return frame,time.monotonic(),items
                with patch.object(self.c.vision,'take_next_pick',return_value=self.prepared()),\
                     patch.object(self.c.vision,'sample',side_effect=fresh):
                    with self.assertRaisesRegex(RuntimeError,'target'):
                        self.c._target()
                self.assertIsNone(self.c.journal.state['target'])

    def test_cached_frame_cannot_be_used_after_lift(self):
        stable=self.prepared()
        with patch.object(self.c.vision,'take_next_pick',return_value=stable),\
             patch.object(self.c.vision,'sample',return_value=(6,stable.last_stamp,[board()])):
            with self.assertRaisesRegex(RuntimeError,'target'):
                self.c._target()
        self.assertIsNone(self.c.journal.state['target'])

    def test_camera_error_cannot_be_hidden_by_prepared_target(self):
        with patch.object(self.c.vision,'take_next_pick',return_value=self.prepared()),\
             patch.object(self.c.vision,'sample',side_effect=RuntimeError('Camera lost')):
            with self.assertRaisesRegex(RuntimeError,'Camera lost'):
                self.c._target()

    def test_wrong_pick_order_or_hold_discards_preparation(self):
        self.c.vision.prepare_next_pick(after_id=1)
        self.assertIsNone(self.c.vision.take_next_pick(after_id=2))
        self.c.vision.prepare_next_pick(after_id=1)
        self.c.hold()
        self.assertIsNone(self.c.vision.take_next_pick(after_id=1))
