"""Laser/vacuum faults are owned by PICKDROP; Python never calls HAND."""
from tests.pickdrop_support import PickDropCase


class LaserH2CycleTests(PickDropCase):
    def test_laser_fault_checkpoint_does_not_infer_vacuum_fault_or_release(self):
        self.c.robot.client.scenario='laser_timeout'
        result=self.run_cycle()
        self.assert_fault(result)
        self.assertIn('B091=2',result['error'])
        self.assertEqual(self.starts(),['PICKDROP'])

    def test_vacuum_fault_checkpoint_does_not_auto_release_or_restart(self):
        self.c.robot.client.scenario='vacuum_timeout'
        result=self.run_cycle()
        self.assert_fault(result)
        self.assertIn('B091=2',result['error'])
        self.assertEqual(self.starts(),['PICKDROP'])
