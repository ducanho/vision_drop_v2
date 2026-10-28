import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from core.settings import load_config
from workflow.cycle import Cycle


class RepeatTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        cfg=load_config()
        cfg['vision'].update(stable_frames=2,stable_seconds=.04)
        cfg['workflow']['empty_seconds']=.15
        cfg['motion'].update(poll_seconds=.01,target_timeout=1,settle_samples=2)
        self.c=Cycle(cfg,True,Path(self.tmp.name)/'execution.json')
        self.c.connect()
        # Feed the simulated scene to the background prefetch as well as sample().
        update=self.c.vision._update_next_pick
        self.prefetch_scene=lambda items:items
        self.prefetch_patch=patch.object(self.c.vision,'_update_next_pick',
            side_effect=lambda frame,stamp,items:update(frame,stamp,self.prefetch_scene(items)))
        self.prefetch_patch.start()
        deadline=time.monotonic()+2
        while not self.c.vision.status()['connected'] and time.monotonic()<deadline:
            time.sleep(.01)

    def tearDown(self):
        self.c.close()
        self.prefetch_patch.stop()
        self.tmp.cleanup()

    def run_loop(self,timeout=10):
        self.c.start('repeat-request-001',self.c.journal.state['revision'])
        self.c.thread.join(timeout=timeout)
        self.assertFalse(self.c.busy)
        return self.c.journal.snapshot()

    def test_two_boards_then_empty_stops_without_third_move(self):
        self.prefetch_scene=lambda items:[] if self.c.stack.snapshot()['stacked_count']>=1 else items
        sample=self.c.vision.sample
        lift=self.c._confirm_pickdrop
        lifts=[]
        prefills=[]
        def observe():
            frame,stamp,items=sample()
            return frame,stamp,[] if self.c.stack.snapshot()['stacked_count']>=2 else items
        def raised():
            lift()
            lifts.append(self.c.robot.pose()[0]['z'])
            prefills.append(self.c.journal.state.get('target_prefilled_frames',0))
        with patch.object(self.c.vision,'sample',side_effect=observe),patch.object(self.c,'_confirm_pickdrop',side_effect=raised):
            s=self.run_loop()
        self.assertEqual(s['state'],'COMPLETE',s['error'])
        self.assertEqual(s['stop_reason'],'no_boards')
        self.assertEqual(lifts,[530,542])
        self.assertEqual(prefills[0],0)
        self.assertGreaterEqual(prefills[1],2)
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],2)
        self.assertEqual(len(set(self.c.stack.data['completed_drop_cycle_ids'])),2)
        self.assertEqual([d for n,d in self.c.robot.client.commands if n=='START'],['PICKDROP']*2)
        self.assertEqual(sum(n=='MOVL' for n,d in self.c.robot.client.commands),0)
        self.assertFalse(any(n=='HOLD' for n,d in self.c.robot.client.commands))

    def test_empty_initial_scene_completes_without_motion(self):
        sample=self.c.vision.sample
        def empty():
            frame,stamp,_=sample()
            return frame,stamp,[]
        with patch.object(self.c.vision,'sample',side_effect=empty):
            s=self.run_loop()
        self.assertEqual((s['state'],s['stop_reason']),('COMPLETE','no_boards'))
        self.assertFalse(any(n in ('MOVL','START','HOLD') for n,d in self.c.robot.client.commands))
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],0)

    def test_repeated_full_cycles_pick_12341_even_when_id_one_reappears(self):
        sample=self.c.vision.sample
        lift=self.c._confirm_pickdrop
        picked=[]
        def next_scene(items):
            count=self.c.stack.snapshot()['stacked_count']+1
            if count>=5:
                return []
            return [{**items[0],'id':i,'instance':100+count if i==1 else i,
                     'xyz':(700+i*40,100,0)} for i in (4,1,3,2)]
        self.prefetch_scene=next_scene
        def observe():
            frame,stamp,items=sample()
            count=self.c.stack.snapshot()['stacked_count']
            if count>=5:
                return frame,stamp,[]
            boards=[{**items[0],'id':i,'instance':100+count if i==1 else i,
                     'xyz':(700+i*40,100,0)} for i in (4,1,3,2)]
            return frame,stamp,boards
        def raised():
            lift()
            picked.append(self.c.journal.state['target']['id'])
        with patch.object(self.c.vision,'sample',side_effect=observe),patch.object(self.c,'_confirm_pickdrop',side_effect=raised):
            s=self.run_loop(timeout=20)
        self.assertEqual(s['state'],'COMPLETE',s['error'])
        self.assertEqual(picked,[1,2,3,4,1])
        self.assertEqual(self.c.stack.snapshot()['last_picked_id'],1)
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],5)

    def test_camera_loss_after_first_board_faults_without_next_pick(self):
        sample=self.c.vision.sample
        lift=self.c._confirm_pickdrop
        def lost_after_lift():
            lift()
            self.c.vision.connected=False
        def observe():
            if self.c.stack.snapshot()['stacked_count']:
                raise RuntimeError('Camera disconnected')
            return sample()
        with patch.object(self.c.vision,'sample',side_effect=observe),patch.object(self.c,'_confirm_pickdrop',side_effect=lost_after_lift):
            s=self.run_loop()
        self.assertEqual(s['state'],'FAULT')
        self.assertIn('Camera disconnected',s['error'])
        self.assertIsNone(s.get('stop_reason'))
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],1)
        self.assertEqual(sum(n=='MOVL' for n,d in self.c.robot.client.commands),0)

    def test_hold_after_lift_prevents_next_pick(self):
        lift=self.c._confirm_pickdrop
        def raised():
            lift()
            self.c.hold()
        with patch.object(self.c,'_confirm_pickdrop',side_effect=raised):
            s=self.run_loop()
        self.assertEqual(s['state'],'HOLD')
        self.assertEqual(self.c.stack.snapshot()['stacked_count'],1)
        self.assertEqual(sum(n=='MOVL' for n,d in self.c.robot.client.commands),0)
