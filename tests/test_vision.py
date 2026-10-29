import math
import unittest
import cv2
import numpy as np
from core.settings import load_config,owned
from vision.calibration import PixelToRobot
from vision.stability import StableTarget
from vision.detector import Detector


def item(id=1,x=800.,instance=1):
    return dict(id=id,instance=instance,xyz=(x,0.,0.),confidence=.9)


class StabilityTests(unittest.TestCase):
    def setUp(self):
        self.s=StableTarget(load_config()['vision'])

    def test_lowest_id_and_two_seconds(self):
        for n in range(10):
            self.assertIsNone(self.s.update(n+1,10+n*.1,[item(2),item(1)],10+n*.1))
        for n in range(10,21):
            result=self.s.update(n+1,10+n*.1,[item(2),item(1)],10+n*.1)
        self.assertEqual(result['id'],1)

    def test_repeated_frame_never_counts(self):
        for n in range(30):
            self.assertIsNone(self.s.update(1,10,[item()],10+n*.1))
        self.assertEqual(len(self.s.samples),1)

    def test_gap_stale_and_uncertain_reset(self):
        for n in range(9):
            self.s.update(n+1,10+n*.1,[item()],10+n*.1)
        self.assertIsNone(self.s.update(10,13,[item()],13))
        self.assertEqual(len(self.s.samples),1)
        self.assertIsNone(self.s.update(11,14,[item()],14+self.s.cfg['max_frame_age']+1))
        self.assertEqual(len(self.s.samples),0)
        self.assertIsNone(self.s.update(12,17,[{**item(),'uncertain':True}],17))

    def test_motion_over_tolerance_resets(self):
        self.s.update(1,10,[item()],10)
        self.s.update(2,10.1,[item(x=800+self.s.cfg['tolerance_mm']+1)],10.1)
        self.assertEqual(len(self.s.samples),1)

    def test_identity_reuse_resets(self):
        self.s.update(1,10,[item()],10)
        self.s.update(2,10.1,[item(instance=2)],10.1)
        self.assertEqual(len(self.s.samples),1)

    def test_low_confidence_and_nan_never_lock(self):
        for n in range(30):
            candidates=[{**item(),'confidence':.1},item(2,x=math.nan)]
            self.assertIsNone(self.s.update(n+1,10+n*.1,candidates,10+n*.1))


class CalibrationTests(unittest.TestCase):
    def test_real_camera_connect_accepts_existing_needs_review_calibration(self):
        from unittest.mock import Mock,patch
        from types import SimpleNamespace
        import ctypes as ct
        import ipaddress
        from vision.pipeline import Vision
        cfg=load_config()
        vision=Vision(cfg,False)
        self.assertEqual(vision.calibration.quality,'needs_review')
        camera=Mock()
        camera.module=SimpleNamespace(MV_GIGE_DEVICE=1,MV_CC_DEVICE_INFO=ct.c_int)
        camera.devices=SimpleNamespace(pDeviceInfo=[0])
        camera.enumerate.return_value=[{'index':0}]
        info=SimpleNamespace(nTLayerType=1,SpecialInfo=SimpleNamespace(
            stGigEInfo=SimpleNamespace(nCurrentIp=int(ipaddress.IPv4Address(cfg['camera_ip'])))))
        with patch('vision.pipeline.Detector') as detector,patch('vision.pipeline.MvsCamera',return_value=camera),\
             patch('ctypes.cast',return_value=SimpleNamespace(contents=info)),patch.object(vision,'_work'):
            vision.connect()
            detector.assert_called_once_with(owned('models/runs/obb_van_v1-2/weights/best.pt'),cfg['vision'])
            camera.connect.assert_called_once_with(0)
            self.assertTrue(vision.connected)
        vision.close()

    def test_actual_copied_calibration_offset_once(self):
        c=PixelToRobot(owned('calibration/eye_to_hand.npz'))
        u,v=c.size[0]/2,c.size[1]/2
        normalized=cv2.undistortPoints(np.array([[[u,v]]],dtype=float),c.K,c.distortion).reshape(2)
        ray=c.T[:3,:3]@np.array([*normalized,1.])
        origin=c.T[:3,3]
        raw=origin+(0-origin[2])/ray[2]*ray
        result=c.convert(u,v,0)
        np.testing.assert_allclose(result-raw,[280,215,0],atol=1e-8)
        with self.assertRaises(ValueError):
            c.convert(-1,0,0)
        with self.assertRaises(ValueError):
            c.check_frame(np.zeros((10,10,3),dtype=np.uint8))

    def test_detector_obb_center_uses_calibration_once(self):
        from unittest.mock import Mock
        d=Detector.__new__(Detector)
        d.config={'imgsz':960,'device':'cpu'}
        box=Mock()
        box.cpu.return_value=box
        box.xyxy=[Mock()]
        box.xyxy[0].tolist.return_value=[10,20,30,60]
        box.xywhr=[Mock()]
        box.xywhr[0].tolist.return_value=[21,41,20,40,.4]
        box.xyxyxyxy=[Mock()]
        box.xyxyxyxy[0].tolist.return_value=[[10,20],[30,20],[30,60],[10,60]]
        box.conf=[Mock()];box.conf[0].item.return_value=.9
        box.cls=[Mock()];box.cls[0].item.return_value=0
        result=Mock(orig_shape=(100,100),obb=[box],boxes=None,names={0:'part'})
        d.model=Mock();d.model.predict.return_value=[result]
        c=Mock(size=(100,100));c.convert.return_value=(1,2,0)
        items=d.detect(np.zeros((100,100,3),dtype=np.uint8),c,{'confidence':.5,'plane_z_mm':0})
        self.assertEqual((items[0]['u'],items[0]['v']),(21,41))
        c.convert.assert_called_once_with(21,41,0)
