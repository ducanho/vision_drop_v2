"""Native-size YOLO detection and copied Eye-to-Hand transform."""
import math
import os


class Detector:
    def __init__(self, model_path, config):
        os.environ.setdefault('YOLO_CONFIG_DIR', str(model_path.parent.parent / 'logs/ultralytics'))
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError('REAL cần ultralytics/PyTorch. Chạy .venv\\Scripts\\python -m pip install -r requirements/real.txt') from exc
        self.model = YOLO(str(model_path))
        self.config = config

    def detect(self, frame, calibration, cfg):
        calibration.check_frame(frame)
        result = self.model.predict(source=frame, conf=cfg['confidence'],
                    imgsz=self.config['imgsz'], device=self.config['device'], verbose=False)[0]
        if tuple(result.orig_shape) != (calibration.size[1], calibration.size[0]):
            raise ValueError('YOLO original image size differs from calibration')
        h, w = frame.shape[:2]
        items = []
        for box in result.boxes:
            bbox = tuple(float(v) for v in box.xyxy[0].cpu().tolist())
            x1, y1, x2, y2 = bbox
            if not all(math.isfinite(v) for v in bbox) or not (0 <= x1 < x2 <= w and 0 <= y1 < y2 <= h):
                continue
            u, v = (x1 + x2) / 2, (y1 + y2) / 2
            item = dict(bbox=bbox, u=u, v=v, confidence=float(box.conf[0].item()),
                        name=result.names[int(box.cls[0].item())])
            try:
                item['xyz'] = tuple(float(v) for v in calibration.convert(u, v, cfg['plane_z_mm']))
            except Exception as exc:
                item['error'] = str(exc)
            items.append(item)
        return items
