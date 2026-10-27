import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock
from core.settings import load_config
from robot.vacuum import read_outputs,write,validate_mapping
from workflow.cycle import Cycle
from workflow.safety_checks import manual_io_ready


class VacuumTests(unittest.TestCase):
    def test_read_confirmed_relay_addresses_without_write_mapping(self):
        cfg=load_config()['vacuum']
        client=Mock()
        client.execute.return_value='6'
        self.assertEqual(read_outputs(client,cfg),([False,True],6))
        client.execute.assert_called_once_with('IOREAD','30050,8')

    def test_bad_readback_addresses_fail_before_io(self):
        client=Mock()
        with self.assertRaises(ValueError):
            read_outputs(client,{'output_contacts':[30050,30061]})
        client.execute.assert_not_called()

    def test_physical_output_is_not_silently_used_as_network_input(self):
        cfg=dict(verified=True,mode='independent',owned_byte=True,start=30050,bits=[0,1],active_high=[True,True])
        with self.assertRaises(RuntimeError):
            validate_mapping(cfg)

    def test_manual_io_does_not_require_servo_or_motion_commissioning(self):
        flags=dict(remote=True,teach=False,running=False,alarm=False,error=False,hold=False,servo=False)
        manual_io_ready(flags)
        with tempfile.TemporaryDirectory() as tmp:
            c=Cycle(load_config(),True,Path(tmp)/'execution.json')
            try:
                c.robot.connect()
                c.refresh()
                c.mock=False
                self.assertFalse(c.cfg['commissioning']['full_cycle'])
                self.assertFalse(c.snapshot()['capabilities']['vacuum_only']['allowed'])
                self.assertTrue(c.snapshot()['capabilities']['manual_vacuum']['allowed'])
                c.vacuum(1,True)
                self.assertEqual(c.robot.vacuum,[True,False])
            finally:
                c.close()

    def test_network_echo_does_not_prove_physical_relay_changed(self):
        cfg=dict(verified=True,mode='independent',owned_byte=True,start=22010,bits=[0,1],
                 active_high=[True,True],output_contacts=[30050,30051])
        network=0
        def execute(name,data=''):
            nonlocal network
            if name=='IOWRITE':
                network=int(data.split(',')[2]);return '0000'
            return str(network) if data=='22010,8' else '0'
        client=Mock();client.execute.side_effect=execute
        with self.assertRaisesRegex(RuntimeError,'OUTPUT relay'):
            write(client,cfg,1,True)
        self.assertEqual(sum(c.args[0]=='IOWRITE' for c in client.execute.call_args_list),1)
