import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from core.settings import load_config
from vision.stability import StableTarget
from workflow.drop import StackStore


class PickOrderTests(unittest.TestCase):
    def choose(self,ids,cursor,instance=1):
        cfg={**load_config()['vision'],'stable_frames':2,'stable_seconds':.05}
        stable=StableTarget(cfg,after_id=cursor)
        items=[dict(id=i,instance=instance,xyz=(700+i*50,0,0),confidence=.9) for i in ids]
        stable.update(1,10,items,10)
        return stable.update(2,10.1,items,10.1)['id']

    def test_reappearing_low_ids_wait_until_higher_ids_are_picked(self):
        cursor=None
        result=[]
        for generation in range(9):
            cursor=self.choose([4,1,3,2],cursor,instance=generation+1)
            result.append(cursor)
        self.assertEqual(result,[1,2,3,4,1,2,3,4,1])

    def test_missing_ids_skipped_and_single_visible_id_remains_pickable(self):
        self.assertEqual(self.choose([1,4],1),4)
        self.assertEqual(self.choose([1,2],4),1)
        self.assertEqual(self.choose([1],1),1)

    def test_failed_commit_does_not_advance_and_restart_keeps_order(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'stack.json'
            store=StackStore(p,load_config()['drop'])
            store.confirm('cycle-one',picked_id=1)
            with patch.object(store,'_save',side_effect=OSError('disk full')):
                with self.assertRaises(OSError):
                    store.confirm('cycle-two',picked_id=2)
            self.assertEqual(store.snapshot()['last_picked_id'],1)
            restored=StackStore(p,load_config()['drop'])
            self.assertEqual(self.choose([1,2,3,4],restored.snapshot()['last_picked_id']),2)
            self.assertFalse(restored.confirm('cycle-one',picked_id=4))
            self.assertEqual(restored.snapshot()['last_picked_id'],1)
