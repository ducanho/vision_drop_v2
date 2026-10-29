"""Offline OBB regression: real Results API, synthetic image/video, no hardware."""
import math
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import cv2
import numpy as np

from core.settings import load_config, owned
from vision.calibration import PixelToRobot
from vision.detector import Detector, board_theta
from vision.stability import StableTarget
from vision.tracker import BoardTracker


class OBBPipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            from ultralytics.engine.results import Results
        except ImportError:
            raise unittest.SkipTest('Optional ultralytics required for real OBB Results fixtures')
        cls.Results = Results

    def detector(self, rows, shape=(640, 960)):
        detector = Detector.__new__(Detector)
        detector.config = dict(imgsz=960, device='cpu')
        detector.model = Mock()
        result = self.Results(np.zeros((*shape, 3), np.uint8), path='offline.png',
                              names={0: 'van'}, obb=np.asarray(rows, dtype=np.float32).reshape(-1, 7))
        detector.model.predict.return_value = [result]
        return detector

    def test_native_960x640_edge_obb_is_not_dropped_or_recentered(self):
        frame = np.zeros((640, 960, 3), np.uint8)
        detector = self.detector([[8, 300, 200, 30, .4, .9, 0]])
        calibration = Mock(size=(960, 640))
        calibration.convert.return_value = (1, 2, 0)
        items = detector.detect(frame, calibration, dict(confidence=.5, plane_z_mm=0))
        self.assertEqual(len(items), 1)  # OBB corners may extend beyond the frame.
        self.assertEqual(items[0]['name'], 'van')
        self.assertEqual(np.asarray(items[0]['corners']).shape, (4, 2))
        np.testing.assert_allclose(np.mean(items[0]['corners'], axis=0), [8., 300.], atol=1e-5)
        calibration.convert.assert_called_once_with(8., 300., 0)
        options = detector.model.predict.call_args.kwargs
        self.assertEqual(options['conf'], .5)

    def test_angle_width_height_representation_changes_keep_id(self):
        tracker = BoardTracker()
        for n in range(30):
            # Equivalent OBB representations must give the old tracker the same AABB.
            w, h, angle = (300, 30, .4) if n % 2 else (30, 300, .4+math.pi/2)
            detector = self.detector([[400, 300, w, h, angle, .9, 0]])
            calibration = Mock(size=(960, 640))
            calibration.convert.return_value = (800, 0, 0)
            items = tracker.update(detector.detect(np.zeros((640, 960, 3), np.uint8),
                                                   calibration, dict(confidence=.5, plane_z_mm=0)), n*.1)
            self.assertEqual((items[0]['id'], items[0]['instance']), (1, 1))
            self.assertFalse(items[0].get('uncertain', False))
            self.assertAlmostEqual(items[0]['theta_deg'], math.degrees(.4)-90, places=4)

    def test_stationary_boards_keep_ids_when_confidence_order_changes(self):
        tracker = BoardTracker()
        for n in range(25):
            rows = [[250,300,150,100,.4,.95 if n%2 else .6,0],
                    [700,300,150,100,.4,.6 if n%2 else .95,0]]
            if n%2:
                rows.reverse()
            detector = self.detector(rows)
            calibration = Mock(size=(960,640))
            calibration.convert.return_value = (800,0,0)
            items = tracker.update(detector.detect(np.zeros((640,960,3),np.uint8),
                                                   calibration,dict(confidence=.5,plane_z_mm=0)),n*.1)
            self.assertEqual([(i['id'],i['u']) for i in items],[(1,250),(2,700)])
            self.assertFalse(any(i.get('uncertain') for i in items))

    def test_equivalent_angle_representations_do_not_change_robot_xy(self):
        calibration = PixelToRobot(owned('calibration/eye_to_hand.npz'))
        tracker = BoardTracker()
        xy = []
        for n, (w, h, angle) in enumerate([(300, 30, .4), (30, 300, .4+math.pi/2)]):
            detector = self.detector([[2000, 1800, w, h, angle, .9, 0]],
                                     (calibration.size[1], calibration.size[0]))
            frame = np.zeros((calibration.size[1], calibration.size[0], 3), np.uint8)
            items = tracker.update(detector.detect(frame, calibration, dict(confidence=.5, plane_z_mm=0)), n*.1)
            xy.append(items[0]['xyz'])
            self.assertEqual((items[0]['id'], items[0]['instance']), (1, 1))
        np.testing.assert_allclose(xy[0], xy[1], atol=1e-8)

    def test_real_ambiguity_blocks_stable(self):
        def board(u):
            return dict(u=u, v=100, bbox=(u-50, 50, u+50, 150), name='van', confidence=.9)
        tracker = BoardTracker()
        tracker.update([board(100), board(120)], 0)
        items = tracker.update([board(110)], .1)
        self.assertTrue(items[0]['uncertain'])

    def test_wrong_model_and_missing_obb_fail_explicitly(self):
        with patch('ultralytics.YOLO') as yolo:
            with self.assertRaisesRegex(FileNotFoundError, 'checkpoint does not exist'):
                Detector(owned('models/nonexistent-obb.pt'), load_config()['vision'])
            yolo.assert_not_called()
        with patch('ultralytics.YOLO', return_value=Mock(task='detect')):
            with self.assertRaisesRegex(ValueError, 'requires an OBB checkpoint'):
                Detector(owned('models/best.pt'), load_config()['vision'])
        detector = self.detector([])
        detector.model.predict.return_value[0].obb = None
        with self.assertRaisesRegex(ValueError, 'no OBB output'):
            detector.detect(np.zeros((640, 960, 3), np.uint8), Mock(size=(960, 640)),
                            dict(confidence=.5, plane_z_mm=0))

    def test_offline_png_video_to_robot_xy_and_stable(self):
        """Image-derived OBB fixtures exercise decoding/geometry; no trained-model claim."""
        calibration = PixelToRobot(owned('calibration/eye_to_hand.npz'))
        cfg = load_config()['vision']
        frame = np.zeros((640, 960, 3), np.uint8)
        cv2.rectangle(frame, (320, 260), (620, 340), (255, 255, 255), -1)
        tracker, stable = BoardTracker(), StableTarget(cfg)
        points = []
        with tempfile.TemporaryDirectory() as folder:
            png, video = Path(folder)/'stationary.png', Path(folder)/'stationary.avi'
            self.assertTrue(cv2.imwrite(str(png), frame))
            writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*'MJPG'), 10, (960, 640))
            self.assertTrue(writer.isOpened())
            try:
                for _ in range(31):
                    writer.write(frame)
            finally:
                writer.release()
            np.testing.assert_array_equal(cv2.imread(str(png)), frame)
            capture = cv2.VideoCapture(str(video))
            try:
                index = 0
                while True:
                    ok, decoded = capture.read()
                    if not ok:
                        break
                    _, mask = cv2.threshold(cv2.cvtColor(decoded, cv2.COLOR_BGR2GRAY), 127, 255, cv2.THRESH_BINARY)
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    (u, v), (w, h), angle = cv2.minAreaRect(max(contours, key=cv2.contourArea))
                    # Test source uses the calibration's actual native resolution.
                    sx, sy = calibration.size[0]/960, calibration.size[1]/640
                    native = cv2.resize(decoded, calibration.size)
                    rows = [[u*sx, v*sy, w*sx, h*sy, math.radians(angle), .9, 0]]
                    detector = self.detector(rows, native.shape[:2])
                    items = tracker.update(detector.detect(native, calibration,
                                           dict(confidence=.5, plane_z_mm=0)), 10+index*.1)
                    target = stable.update(index+1, 10+index*.1, items, 10+index*.1)
                    points.append(items[0]['xyz'])
                    self.assertEqual((items[0]['id'], items[0]['instance']), (1, 1))
                    if index < 20:
                        self.assertIsNone(target)
                    index += 1
                self.assertEqual(index, 31)
                self.assertIsNotNone(target)
                np.testing.assert_allclose(points, np.tile(points[0], (31, 1)), atol=1e-8)
            finally:
                capture.release()
        # A 960x640 real input must never be silently resized into this calibration.
        with self.assertRaises(ValueError):
            calibration.check_frame(frame)


class BoardThetaTests(unittest.TestCase):
    def test_vertical_right_left_and_corner_order(self):
        for expected in (0, 15.2, -15.2, 89.9, -89.9):
            a = math.radians(expected)
            axis = np.array([math.sin(a), -math.cos(a)]) * 150
            across = np.array([math.cos(a), math.sin(a)]) * 20
            corners = np.array([axis+across, axis-across, -axis-across, -axis+across])
            for order in (corners, corners[::-1], np.roll(corners, 1, axis=0)):
                self.assertAlmostEqual(board_theta(order), expected)
                self.assertAlmostEqual(90-board_theta(order), 90-expected)

    def test_square_and_degenerate_have_no_angle(self):
        self.assertIsNone(board_theta([(0, 0), (10, 0), (10, 10), (0, 10)]))
        self.assertIsNone(board_theta([(0, 0)]*4))


class StabilityContractTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config()['vision']
        self.stable = StableTarget(self.cfg)

    def item(self, x=800, **extra):
        return dict(id=1, instance=1, xyz=(x, 0, 0), confidence=.9, **extra)

    def test_fifty_mm_boundary_and_real_motion_reset(self):
        self.assertEqual((self.cfg['stable_frames'], self.cfg['stable_seconds'], self.cfg['tolerance_mm']), (10, 2, 50))
        self.stable.update(1, 10, [self.item()], 10)
        self.stable.update(2, 10.1, [self.item(850)], 10.1)
        self.assertEqual(len(self.stable.samples), 2)
        self.stable.update(3, 10.2, [self.item(850.01)], 10.2)
        self.assertEqual(len(self.stable.samples), 1)

    def test_missing_low_confidence_uncertain_and_gap_reset(self):
        for n, (items, count) in enumerate([
                ([self.item()], 1),
                ([], 0),
                ([{**self.item(), 'confidence': .1}], 0),
                ([self.item(uncertain=True)], 0),
                ([self.item()], 1),
                ([self.item()], 1)]):
            stamp = 10+n
            self.stable.update(n+1, stamp, items, stamp)
            self.assertEqual(len(self.stable.samples), count)

    def test_small_jitter_returns_measured_xy_without_smoothing(self):
        for n in range(22):
            x = 800 + (-1)**n * 2
            result = self.stable.update(n+1, 10+n*.1, [self.item(x)], 10+n*.1)
        self.assertEqual(result['xyz'][0], x)


class MockOBBPreviewTests(unittest.TestCase):
    def test_preview_unwraps_horizontal_boundary_and_keeps_instances_separate(self):
        from vision.pipeline import Vision
        vision = Vision(load_config(), mock=False)
        frame = np.zeros((540, 960, 3), np.uint8)
        vision.camera = Mock()
        vision.detector = Mock()
        angles = [89., -89., 89., -89.]
        sequence = 0
        labels = []

        def sample():
            nonlocal sequence
            sequence += 1
            return sequence, time.monotonic(), frame

        def detect(*args):
            return [dict(name='van', bbox=(100, 100, 400, 150), u=250, v=125,
                         xyz=(800, 0, 0), confidence=.97, theta_deg=angles[sequence-1]),
                    dict(name='van', bbox=(600, 100, 900, 150), u=750, v=125,
                         xyz=(1000, 0, 0), confidence=.97, theta_deg=-15.2)]

        def draw(image, label, *args):
            labels.append(label)
            if len(labels) == 8:
                vision.stop.set()
            return image

        vision.camera.sample.side_effect = sample
        vision.detector.detect.side_effect = detect
        with patch('socket.create_connection', side_effect=AssertionError('Offline only')), \
             patch('vision.pipeline.cv2.putText', side_effect=draw):
            vision._work()
        self.assertEqual(len(labels), 8, vision.error)
        values = [float(label.split('theta ')[1].split()[0]) for label in labels[::2]]
        self.assertTrue(all(abs(a-b) < 1 for a,b in zip(values, values[1:])), values)
        self.assertTrue(all('theta -15.2 deg RZ 105.2 deg' in label for label in labels[1::2]))
        self.assertEqual([i['id'] for i in vision.items], [1, 2])

    def test_rotated_preview_and_stable_pose_at_production_dwell(self):
        try:
            from ultralytics.engine.results import Results
        except ImportError:
            self.skipTest('Optional ultralytics required for OBB fixture')
        from vision.pipeline import Vision
        vision = Vision(load_config(), mock=False)
        width,height = vision.calibration.size
        frame = np.zeros((height,width,3),np.uint8)
        detector = Detector.__new__(Detector)
        detector.config = load_config()['vision']
        detector.model = Mock()
        detector.model.predict.return_value = [Results(frame,path='offline.png',names={0:'van'},
            obb=np.array([[2000,1800,1000,300,.4,.9,0]],dtype=np.float32))]
        vision.detector = detector
        vision.camera = Mock()
        sequence = 0
        def sample():
            nonlocal sequence
            sequence += 1
            return sequence,time.monotonic(),frame
        vision.camera.sample.side_effect = sample
        expected = vision.calibration.convert(2000,1800,0)
        with patch('socket.create_connection', side_effect=AssertionError('Offline only')), \
             patch('vision.pipeline.MvsCamera', side_effect=AssertionError('No hardware')), \
             patch('vision.pipeline.cv2.polylines', wraps=cv2.polylines) as draw, \
             patch('vision.pipeline.cv2.putText', wraps=cv2.putText) as text:
            try:
                # Run only the worker with fake sensor frames; never call REAL connect.
                vision.connected = True
                vision.thread = threading.Thread(target=vision._work,daemon=True)
                vision.prepare_next_pick(after_id=None, cycle_id='offline-mock')
                vision.thread.start()
                deadline = time.monotonic()+5
                while time.monotonic() < deadline and vision.prefetch_status()['state'] != 'READY':
                    time.sleep(.02)
                state = vision.prefetch_status()
                self.assertEqual(state['state'], 'READY', vision.error)
                self.assertEqual(state['target']['id'], 1)
                self.assertEqual(state['target']['pose']['rz'], 90)
                self.assertAlmostEqual(vision.items[0]['theta_deg'], math.degrees(.4)-90, places=4)
                label = text.call_args.args[1]
                self.assertIn('theta -67.1 deg RZ 157.1 deg', label)
                self.assertGreaterEqual(len(vision._next_target.samples), 10)
                self.assertGreaterEqual(vision._next_target.last_stamp-vision._next_target.samples[0][0], 2)
                image = cv2.imdecode(np.frombuffer(vision.preview, np.uint8), cv2.IMREAD_COLOR)
                self.assertEqual(image.shape[:2], (640, 960))
                polygon = draw.call_args.args[1][0]
                self.assertEqual(polygon.shape, (4, 2))
                self.assertNotEqual(polygon[0, 1], polygon[1, 1])  # Rotated edge.
                np.testing.assert_allclose(vision.items[0]['xyz'],expected,atol=1e-8)
            finally:
                vision.close()


if __name__ == '__main__':
    unittest.main()
