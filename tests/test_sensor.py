import unittest
from unittest.mock import Mock,patch
from core.settings import load_config
from robot.io import laser_sample
from robot.controller import Controller
from robot.protocol import ProtocolError


class SensorTests(unittest.TestCase):
    def test_pressure_in2_in3_are_independent_of_laser_and_outputs(self):
        robot=Controller(load_config(),True)
        original=robot.client.execute
        for raw,expected in ((0,[False,False]),(2,[True,False]),(4,[False,True]),(6,[True,True]),(7,[True,True])):
            with self.subTest(raw=raw),patch.object(robot.client,'execute',side_effect=lambda name,data='': str(raw) if name=='IOREAD' and data=='20050,8' else original(name,data)):
                sample=robot.observe()
            self.assertEqual(sample['vacuum_inputs'],expected)
            self.assertEqual(sample['laser'],bool(raw&1))
            self.assertEqual(sample['vacuum_input_contacts'],[20051,20052])
        self.assertFalse(any(n in ('IOWRITE','START','MOVL') for n,d in robot.client.commands))

    def test_unverified_mapping_still_reads_actual_byte_for_display(self):
        cfg=load_config()['laser']
        self.assertFalse(cfg['verified'])
        client=Mock()
        client.execute.return_value='4'
        self.assertEqual(laser_sample(client,cfg),(False,4))
        client.execute.assert_called_once_with('IOREAD','20050,8')
        client.execute.return_value='5'
        self.assertEqual(laser_sample(client,cfg),(True,5))

    def test_active_low(self):
        cfg={**load_config()['laser'],'active_high':False}
        client=Mock();client.execute.return_value='0'
        self.assertEqual(laser_sample(client,cfg),(True,0))

    def test_bad_or_missing_reply_is_not_off(self):
        client=Mock()
        for reply in ('','ERROR: 2100','256','1,2','x'):
            client.execute.return_value=reply
            with self.subTest(reply=reply),self.assertRaises(RuntimeError):
                laser_sample(client,load_config()['laser'])

    def test_ioread_failure_preserves_robot_status_and_position(self):
        robot=Controller(load_config(),True)
        original=robot.client.execute
        def fail(name,data=''):
            if name=='IOREAD':
                raise ProtocolError('ERROR: [IOREAD] is not successful(2100).')
            return original(name,data)
        with patch.object(robot.client,'execute',fail):
            data=robot.observe()
        self.assertTrue(data['connected'])
        self.assertTrue(data['flags']['servo'])
        self.assertEqual(data['position']['z'],500)
        self.assertIsNone(data['laser'])
        self.assertIsNone(data['laser_raw_byte'])
        self.assertEqual(data['vacuum_inputs'],[None,None])
        self.assertIn('2100',data['read_errors']['laser'])

    def test_sensor_reads_even_if_position_reply_fails(self):
        robot=Controller(load_config(),True)
        robot.client.laser=True
        original=robot.client.execute
        def fail(name,data=''):
            if name=='RPOSC':
                raise ProtocolError('Connection closed')
            return original(name,data)
        with patch.object(robot.client,'execute',fail):
            data=robot.observe()
        self.assertTrue(data['connected'])
        self.assertIsNone(data['position'])
        self.assertTrue(data['laser'])
        self.assertEqual(data['laser_raw_byte'],1)
