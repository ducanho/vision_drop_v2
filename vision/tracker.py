"""Stable board IDs by bbox association; release an unseen ID after 10 seconds."""
import math
import time


class BoardTracker:
    def __init__(self, missing_timeout=10.0):
        self.missing_timeout = missing_timeout
        self.tracks = {}
        self.generation = 0

    @staticmethod
    def match_score(previous, current):
        if previous["name"] != current["name"]:
            return None
        a, b = previous["bbox"], current["bbox"]
        aw, ah = max(a[2] - a[0], 1), max(a[3] - a[1], 1)
        bw, bh = max(b[2] - b[0], 1), max(b[3] - b[1], 1)
        if max(aw / bw, bw / aw, ah / bh, bh / ah) > 2:
            return None
        overlap = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
        iou = overlap / (aw * ah + bw * bh - overlap)
        distance = math.hypot((a[0] + a[2] - b[0] - b[2]) / 2,
                              (a[1] + a[3] - b[1] - b[3]) / 2)
        relative_distance = distance / max(math.hypot(aw, ah), math.hypot(bw, bh))
        if iou < .1 and relative_distance > .5:
            return None
        return iou + .2 * (1 - min(relative_distance, 1))

    def update(self, detections, now=None):
        now = time.monotonic() if now is None else now
        # Hidden tracks reserve their IDs, but never produce stale robot targets.
        self.tracks = {board_id: track for board_id, track in self.tracks.items()
                       if now < track["last_seen"] + self.missing_timeout}
        candidates = []
        for board_id, track in self.tracks.items():
            for index, item in enumerate(detections):
                score = self.match_score(track, item)
                if score is not None:
                    candidates.append((-score, board_id, index))
        assigned_ids, assigned_indices = set(), set()
        for score, board_id, index in candidates:
            if any((other_id == board_id or other_index == index)
                   and (other_id, other_index) != (board_id, index)
                   and abs(other_score - score) < .05
                   for other_score, other_id, other_index in candidates):
                detections[index]['uncertain'] = True
        for _, board_id, index in sorted(candidates):
            if board_id not in assigned_ids and index not in assigned_indices:
                detections[index]["id"] = board_id
                assigned_ids.add(board_id)
                assigned_indices.add(index)
        # Same-frame arrivals have no temporal order: assign left-to-right, then top-to-bottom.
        new_indices = sorted((i for i in range(len(detections)) if i not in assigned_indices),
                             key=lambda i: (detections[i]["u"], detections[i]["v"]))
        reserved_ids = set(self.tracks)
        for index in new_indices:
            board_id = 1
            while board_id in reserved_ids:
                board_id += 1
            detections[index]["id"] = board_id
            reserved_ids.add(board_id)
        for item in detections:
            prior = self.tracks.get(item["id"])
            if prior is None:
                self.generation += 1
            item["instance"] = prior["instance"] if prior else self.generation
            self.tracks[item["id"]] = dict(bbox=item["bbox"], name=item["name"], last_seen=now,
                                           instance=item["instance"])
        return sorted(detections, key=lambda item: item["id"])
