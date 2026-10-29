import threading
import time
import logging
from copy import deepcopy
import cv2
import numpy as np
from core.settings import owned, fingerprint
from .calibration import PixelToRobot
from .detector import Detector
from .camera import MvsCamera
from .tracker import BoardTracker
from .stability import StableTarget


class Vision:
    def __init__(self, cfg, mock):
        self.cfg, self.mock = cfg, mock
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.thread = None
        self.camera = None
        self.detector = None
        self.calibration = PixelToRobot(owned('calibration/eye_to_hand.npz'))
        # Verify the second, copied intrinsic asset is readable as well.
        with np.load(owned('calibration/camera_intrinsic_2.npz'), allow_pickle=False) as intrinsic:
            self.intrinsic_keys = list(intrinsic.files)
            if not np.allclose(intrinsic['K'],self.calibration.K) or not np.allclose(intrinsic['distortion'].reshape(-1),self.calibration.distortion):
                raise RuntimeError('Intrinsic và Eye-to-Hand không khớp')
        self.connected = False
        self.error = ''
        self.preview = None
        self.frame_id, self.stamp, self.items, self.fps = 0, 0, [], 0
        self.tracker = BoardTracker()
        self._preview_theta = {}  # Display only, keyed by tracker instance.
        self._next_target = None
        self._prefetch = dict(state='IDLE', cycle_id=None)

    def preparation_identity(self):
        c = self.calibration
        return (fingerprint(self.cfg), id(c), c.T.tobytes(), c.K.tobytes(),
                c.distortion.tobytes(), c.path.read_bytes())

    def _update_next_pick(self, frame_id, stamp, items):
        if self._next_target is None:
            return
        if self._prefetch['identity'] != self.preparation_identity():
            self.clear_next_pick('calibration/config changed')
            return
        if self._prefetch['state']=='READY':
            return  # Keep the first stable post-B090 coordinates until dispatch.
        if stamp <= self._prefetch['started_at']:
            return
        target = self._next_target.update(frame_id, stamp, items, time.monotonic())
        was_ready = self._prefetch['state'] == 'READY'
        self._prefetch.update(state='READY' if target else 'STABILIZING',
                              target=target, frame_id=frame_id, stamp=stamp)
        if target and not was_ready:
            settings=self.cfg['settings']
            target['pose']=dict(x=target['xyz'][0],y=target['xyz'][1],z=settings['z_safe'],
                                rx=settings['rx'],ry=settings['ry'],rz=settings['rz'])
            self._prefetch['ready_at'] = time.monotonic()
            logging.getLogger('cycle').info('%s NEXT_TARGET_READY id=%s XY=%s',
                self._prefetch['cycle_id'], target['id'], target['xyz'][:2])

    def prefetch_status(self):
        with self.lock:
            result=deepcopy({k:v for k,v in self._prefetch.items() if k!='identity'})
            result['age_seconds']=time.monotonic()-result['stamp'] if 'stamp' in result else None
            return result

    def consume_next_pick(self):
        with self.lock:
            self._next_target=None
            self._prefetch['state']='CONSUMED'

    def connect(self):
        if self.connected:
            return
        if self.thread and self.thread.is_alive():
            raise RuntimeError('Camera worker vẫn đang dừng')
        self.clear_next_pick()
        if not self.mock:
            self.detector = Detector(owned('models/runs/obb_van_v1-2/weights/best.pt'), self.cfg['vision'])
            if self.camera:
                self.camera.close()
            self.camera = MvsCamera(self.cfg['vision']['sdk_root'])
            devices = self.camera.enumerate()
            # Match actual GigE IP, never silently select a different camera.
            import ctypes as ct
            import ipaddress
            m = self.camera.module
            matches = []
            for d in devices:
                info = ct.cast(self.camera.devices.pDeviceInfo[d['index']], ct.POINTER(m.MV_CC_DEVICE_INFO)).contents
                if info.nTLayerType == m.MV_GIGE_DEVICE and str(ipaddress.IPv4Address(info.SpecialInfo.stGigEInfo.nCurrentIp)) == self.cfg['camera_ip']:
                    matches.append(d['index'])
            if len(matches) != 1:
                raise RuntimeError('Không tìm thấy đúng một camera GigE tại IP đã cấu hình')
            self.camera.connect(matches[0])
        self.stop.clear()
        self.connected, self.error = True, ''
        self.thread = threading.Thread(target=self._work, daemon=True, name='vision')
        self.thread.start()

    def _work(self):
        last = -1
        started = time.monotonic()
        while not self.stop.wait(.1):
            try:
                with self.lock:
                    settings = deepcopy(self.cfg['settings'])
                if self.mock:
                    frame = np.full((540, 960, 3), 25, dtype=np.uint8)
                    stamp, frame_id = time.monotonic(), self.frame_id+1
                    items = [dict(name='MOCK', confidence=.97, bbox=(150+300*i,200,250+300*i,300),
                                  u=200+300*i, v=250, xyz=(850.+200*i, 100., settings['plane_z_mm'])) for i in range(2)]
                else:
                    frame_id, stamp, frame = self.camera.sample()
                    if frame_id == last:
                        continue
                    items = self.detector.detect(frame, self.calibration, {**settings, 'confidence':self.cfg['vision']['confidence']})
                items = self.tracker.update(items, stamp)
                self._preview_theta = {key: value for key, value in self._preview_theta.items()
                                       if key in {t['instance'] for t in self.tracker.tracks.values()}}
                last = frame_id
                scale = min(1, 960/frame.shape[1])
                image = cv2.resize(frame, (round(frame.shape[1]*scale), round(frame.shape[0]*scale)))
                for item in items:
                    a,b,c,d = [round(v*scale) for v in item['bbox']]
                    if 'corners' in item:
                        points = np.rint(np.asarray(item['corners'])*scale).astype(np.int32)
                        cv2.polylines(image,[points],True,(80,220,140),2)
                    else:
                        cv2.rectangle(image,(a,b),(c,d),(80,220,140),2)
                    xy = item.get('xyz', (0,0))
                    label = f'ID {item["id"]} {item["confidence"]:.0%} X {xy[0]:.1f} Y {xy[1]:.1f}'
                    theta = item.get('theta_deg')
                    previous = self._preview_theta.get(item['instance'])
                    if theta is not None:
                        if previous is not None:
                            # Nearest 180-degree equivalent, then damp frame jitter.
                            theta = previous + .3 * ((theta-previous+90) % 180-90)
                        self._preview_theta[item['instance']] = theta
                    else:
                        theta = previous
                    if theta is not None:
                        label += f' theta {theta:+.1f} deg RZ {90-theta:.1f} deg'
                    label_width = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .5, 1)[0][0]
                    cv2.putText(image,label,
                                (max(0,min(a,image.shape[1]-label_width-1)),max(20,b-10)),
                                cv2.FONT_HERSHEY_SIMPLEX,.5,(80,220,140),1)
                ok, jpeg = cv2.imencode('.jpg', image)
                with self.lock:
                    self.fps = 1/max(time.monotonic()-started,.001)
                    started = time.monotonic()
                    self.frame_id,self.stamp,self.items = frame_id,stamp,items
                    if self._next_target is not None:
                        self._update_next_pick(frame_id,stamp,items)
                    if ok:
                        self.preview = jpeg.tobytes()
            except Exception as exc:
                if self.frame_id == 0 and time.monotonic()-started < 3:
                    continue
                with self.lock:
                    self.connected,self.error,self.items = False,str(exc),[]
                    self.clear_next_pick('camera disconnected')
                self.stop.set()

    def prepare_next_pick(self,after_id,cycle_id=None):
        with self.lock:
            self._next_target=StableTarget(deepcopy(self.cfg['vision']),after_id=after_id)
            self._prefetch=dict(state='DETECTING',cycle_id=cycle_id,
                                started_at=time.monotonic(),identity=self.preparation_identity())

    def take_next_pick(self,after_id):
        with self.lock:
            target=self._next_target
            self._next_target=None
            if target is not None and self._prefetch['identity'] != self.preparation_identity():
                self.clear_next_pick('calibration/config changed')
                return None
            if target is not None and time.monotonic()-target.last_stamp > self.cfg['vision']['max_frame_age']:
                self.clear_next_pick('stale target')
                return None
            if target is not None and target.after_id==after_id:
                self._prefetch['state']='VALIDATING'
                return deepcopy(target)
            self.clear_next_pick('pick order/context changed')
            return None

    def take_ready_pick(self,after_id,cycle_id):
        """Take the latched result once; no new frame or detection is required."""
        with self.lock:
            if self._prefetch['state']!='READY':
                return None
            if (self._prefetch.get('cycle_id')!=cycle_id or self._next_target is None or
                    self._next_target.after_id!=after_id):
                self.clear_next_pick('pick order/context changed')
                return None
            if self._prefetch['identity']!=self.preparation_identity():
                self.clear_next_pick('calibration/config changed')
                return None
            target=deepcopy(self._prefetch['target'])
            target.update(frame_id=self._prefetch['frame_id'],
                          prepared_frames=len(self._next_target.samples))
            self._next_target=None
            self._prefetch['state']='PREPARED'
            return target

    def clear_next_pick(self,reason=None):
        with self.lock:
            self._next_target=None
            self._prefetch=dict(state='INVALID' if reason else 'IDLE',cycle_id=None,reason=reason)

    def sample(self):
        with self.lock:
            if not self.connected or time.monotonic()-self.stamp > self.cfg['vision']['max_frame_age']:
                raise RuntimeError(self.error or 'Camera không có frame mới')
            return self.frame_id, self.stamp, deepcopy(self.items)

    def status(self):
        with self.lock:
            return dict(connected=self.connected and time.monotonic()-self.stamp <= self.cfg['vision']['max_frame_age'],
                        error=self.error, fps=round(self.fps,1), detections=deepcopy(self.items),
                        frame_id=self.frame_id, simulated=self.mock,
                        model_loaded=self.detector is not None, calibration=self.calibration.quality,
                        calibration_warning=self.calibration.quality_message() if self.calibration.quality != 'residual_checks_passed' else '',
                        coordinates_verified=self.calibration.quality == 'residual_checks_passed')

    def close(self):
        self.clear_next_pick()
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=5)
            if self.thread.is_alive():
                raise RuntimeError('Vision worker chưa dừng')
        if self.camera:
            self.camera.close()
        self.connected = False
