"""Stability uses distinct sensor frames and capture time, never HMI polls."""
import math
from copy import deepcopy


class StableTarget:
    def __init__(self, cfg, after_id=None):
        self.cfg = cfg
        self.after_id = after_id
        self.last_frame = -1
        self.last_stamp = 0
        self.samples = []
        self.identity = None

    def update(self, frame, stamp, items, now):
        if frame <= self.last_frame or stamp <= self.last_stamp:
            return None
        gap = stamp-self.last_stamp
        self.last_frame, self.last_stamp = frame, stamp
        if not 0 <= now-stamp <= self.cfg['max_frame_age']:
            self.samples.clear()
            return None
        candidates = [i for i in items if not i.get('uncertain') and 'xyz' in i and
                      i['confidence'] >= self.cfg['confidence'] and all(math.isfinite(v) for v in i['xyz'])]
        if not candidates:
            self.samples.clear()
            self.identity = None
            return None
        # Finish higher IDs first, then wrap to the smallest visible ID.
        item = min(candidates, key=lambda i: (self.after_id is not None and i['id']<=self.after_id, i['id']))
        identity = (item['id'], item['instance'])
        if identity != self.identity or gap > self.cfg['max_frame_gap'] or any(
                math.dist(item['xyz'][:2], sample[1]['xyz'][:2]) > self.cfg['tolerance_mm'] for sample in self.samples):
            self.samples.clear()
        self.identity = identity
        self.samples.append((stamp, deepcopy(item)))
        # Keep bounded history while retaining a full dwell window.
        while len(self.samples) > self.cfg['stable_frames'] and stamp-self.samples[1][0] >= self.cfg['stable_seconds']:
            self.samples.pop(0)
        if len(self.samples) >= self.cfg['stable_frames'] and stamp-self.samples[0][0] >= self.cfg['stable_seconds']:
            return deepcopy(item)
        return None
