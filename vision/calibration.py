"""Existing pixel-to-Base ray/plane algorithm; no robot communication."""
import json
from pathlib import Path

import cv2
import numpy as np

class InputError(ValueError):
    pass


PROJECT = Path(__file__).resolve().parent
OUTPUT_OFFSET_X_MM = 280.0
OUTPUT_OFFSET_Y_MM = 215.0  # Previous +200 mm, increased by +15 mm.


class PixelToRobot:
    def __init__(self, filename):
        self.path = Path(filename).resolve()
        with np.load(self.path, allow_pickle=False) as data:
            self.T = np.array(data["T_base_camera"], dtype=float)
            self.K = np.array(data["K"], dtype=float)
            self.distortion = np.array(data["distortion"], dtype=float).reshape(-1)
            self.size = (int(data["image_width"].item()), int(data["image_height"].item()))
            self.quality = str(data["quality_status"].item()) if "quality_status" in data else "unknown"
            self.report = json.loads(str(data["report_json"].item())) if "report_json" in data else {}
        if self.T.shape != (4, 4) or not np.isfinite(self.T).all():
            raise InputError("T_base_camera phải là ma trận 4x4 hữu hạn.")
        rotation = self.T[:3, :3]
        if (not np.allclose(self.T[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5, rtol=0)
                or not np.isclose(np.linalg.det(rotation), 1, atol=1e-5, rtol=0)):
            raise InputError("T_base_camera không phải rigid transform hợp lệ.")
        if (self.K.shape != (3, 3) or not np.isfinite(self.K).all()
                or self.K[0, 0] <= 0 or self.K[1, 1] <= 0
                or not np.allclose(self.K[2], [0, 0, 1])
                or self.distortion.size not in (4, 5, 8, 12, 14)
                or not np.isfinite(self.distortion).all() or min(self.size) <= 0):
            raise InputError("Intrinsic/kích thước ảnh không hợp lệ.")
        units = self.report.get("units", {}).get("translation")
        if units != "mm":
            raise InputError("Kết quả calib phải dùng đơn vị mm.")
        if self.report.get('T_base_camera_definition') != 'Camera -> robot Base':
            raise InputError('Calibration thiếu định nghĩa Camera -> robot Base đã xác minh.')

    def quality_message(self):
        summary = self.report.get("diagnostics", {}).get("residual_summary", {})
        message = f"Calib: {self.quality}"
        if summary:
            message += (f" | outlier {summary.get('outlier_count')}/{summary.get('samples_checked')}"
                        f" | RMS XYZ {summary.get('translation_error_mm', {}).get('rms', 0):.2f} mm"
                        f" | RMS ảnh {summary.get('reprojection_rms_px', {}).get('rms', 0):.2f} px")
        if self.quality != "residual_checks_passed":
            message += "\nCẢNH BÁO: kết quả cần kiểm tra; tọa độ chưa được xác nhận chính xác."
        return message

    def check_frame(self, frame):
        height, width = frame.shape[:2]
        if (width, height) != self.size:
            raise InputError(f"Ảnh {width}x{height} khác calib {self.size[0]}x{self.size[1]}. "
                             "Đặt lại camera/ROI; không resize ảnh đầu vào để khớp.")

    def convert(self, u, v, plane_z):
        if not np.isfinite([u, v, plane_z]).all():
            raise InputError("Pixel và Z phải là số hữu hạn.")
        if not (0 <= u < self.size[0] and 0 <= v < self.size[1]):
            raise InputError("Pixel nằm ngoài ảnh gốc.")
        # Input là pixel ảnh GỐC có distortion, không phải ảnh đã undistort.
        xy = cv2.undistortPoints(np.array([[[u, v]]], dtype=np.float64),
                                 self.K, self.distortion).reshape(2)
        direction = self.T[:3, :3] @ np.array([xy[0], xy[1], 1.0])
        origin = self.T[:3, 3]
        if not np.isfinite(direction).all() or abs(direction[2]) < 1e-9:
            raise InputError("Tia nhìn song song mặt phẳng Z; không xác định giao điểm.")
        depth = (plane_z - origin[2]) / direction[2]
        if depth <= 0:
            raise InputError("Mặt phẳng nằm phía sau camera theo tia nhìn. Kiểm tra Z và calib.")
        point = origin + depth * direction
        if not np.isfinite(point).all():
            raise InputError("Giao điểm không hữu hạn.")
        point[2] = plane_z
        # Bù đầu ra theo trục Base sau khi giao tia nhìn với mặt phẳng.
        point[0] += OUTPUT_OFFSET_X_MM
        point[1] += OUTPUT_OFFSET_Y_MM
        return point
