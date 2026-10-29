"""Native-size YOLO OBB inference and unchanged Eye-to-Hand transform."""
import math
import os
from pathlib import Path


def board_theta(corners):
    """Long axis relative to image vertical; top leaning right is positive.

    Board orientation is axial (modulo 180 degrees), independent of OBB
    width/height ordering and corner start index. Near-square boxes have no
    reliable long axis, so omit their angle rather than flip by 90 degrees.
    """
    edges = [(corners[(i+1) % 4][0]-corners[i][0],
              corners[(i+1) % 4][1]-corners[i][1]) for i in range(4)]
    lengths = [math.hypot(*edge) for edge in edges]
    if min(lengths) <= 0 or max(lengths) / min(lengths) < 1.05:
        return None
    dx, dy = edges[max(range(4), key=lambda i: lengths[i])]
    return (math.degrees(math.atan2(dx, -dy)) + 90) % 180 - 90


class Detector:
    def __init__(self, model_path, config):
        model_path = Path(model_path)
        if not model_path.is_file():
            raise FileNotFoundError(f'YOLO OBB checkpoint does not exist: {model_path}')
        os.environ.setdefault('YOLO_CONFIG_DIR', str(Path(__file__).resolve().parent.parent / 'logs/ultralytics'))
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError('REAL cần ultralytics/PyTorch. Chạy .venv\\Scripts\\python -m pip install -r requirements/real.txt') from exc
        self.model = YOLO(str(model_path))
        if self.model.task != 'obb':
            raise ValueError(f'YOLO OBB v2 requires an OBB checkpoint; {model_path} has task={self.model.task}')
        self.config = config

    def detect(self, frame, calibration, cfg):
        calibration.check_frame(frame)
        result = self.model.predict(source=frame, conf=cfg['confidence'],
                    imgsz=self.config['imgsz'], device=self.config['device'], verbose=False)[0]
        if tuple(result.orig_shape) != (calibration.size[1], calibration.size[0]):
            raise ValueError('YOLO original image size differs from calibration')
        h, w = frame.shape[:2]
        items = []
        if result.obb is None:
            raise ValueError('YOLO returned no OBB output; Detect fallback is disabled')
        for box in result.obb:
            box = box.cpu()
            # Ultralytics already maps xywhr back to orig_shape after letterboxing.
            # Never scale these pixels again or use a clipped AABB center.
            u, v, width, height, angle = (float(n) for n in box.xywhr[0].tolist())
            confidence = float(box.conf[0].item())
            class_id = int(box.cls[0].item())
            corners = tuple(tuple(float(n) for n in point) for point in box.xyxyxyxy[0].tolist())
            bbox = tuple(float(v) for v in box.xyxy[0].tolist())
            if (not all(math.isfinite(n) for n in (*bbox, u, v, width, height, angle, confidence,
                                                  *(n for point in corners for n in point)))
                    or width <= 0 or height <= 0 or not (0 <= u < w and 0 <= v < h)
                    or not cfg['confidence'] <= confidence <= 1):
                continue
            item = dict(bbox=bbox, u=u, v=v, corners=corners,
                        confidence=confidence, name=result.names[class_id],
                        theta_deg=board_theta(corners))
            try:
                item['xyz'] = tuple(float(v) for v in calibration.convert(u, v, cfg['plane_z_mm']))
            except Exception as exc:
                item['error'] = str(exc)
            items.append(item)
        return items
