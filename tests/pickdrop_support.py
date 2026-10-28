"""Offline fixture; constructing this fixture cannot create a real client."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from core.settings import load_config
from workflow.cycle import Cycle


class PickDropCase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.cfg = load_config()
        self.cfg['workflow']['repeat_until_empty'] = False
        self.cfg['motion'].update(poll_seconds=.01, settle_samples=2)
        self.cfg['drop']['job_timeout'] = 1.1
        self.path = Path(self.tmp.name)/'execution.json'
        self.c = Cycle(self.cfg, True, self.path)
        self.addCleanup(lambda: self.c.close())
        self.c.robot.connect()

    def run_cycle(self, cid='pickdrop-001'):
        self.c.journal.update(state='SYSTEM_CHECK', cycle_id=cid, pending=None,
                              outcome_unknown=False, job_start_attempted=False,
                              target=dict(id=1, xyz=(850,100,0)))
        target = dict(x=850, y=100, z=500, rx=180, ry=0, rz=90)
        with patch.object(self.c, '_target', return_value=target):
            self.c._run_once('full_cycle')
        return self.c.journal.snapshot()

    def starts(self):
        return [d for n,d in self.c.robot.client.commands if n == 'START']

    def assert_fault(self, result):
        self.assertEqual(result['state'], 'FAULT', result.get('error'))
        self.assertEqual(self.c.stack.snapshot()['stacked_count'], 0)
        self.assertNotIn('H2OFF', self.starts())
