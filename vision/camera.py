"""Optional camera adapter using only the installed Hikrobot MVS SDK/wrapper."""
import ctypes as ct
import importlib
import importlib.abc
import importlib.machinery
import os
import struct
import sys
import threading
import time
from pathlib import Path
import cv2
import numpy as np
from .calibration import InputError


class _InstalledSdkFinder(importlib.abc.MetaPathFinder):
    """Load the installed vendor wrapper without modifying sys.path."""
    def __init__(self, folder):
        self.folder = folder
        self.names = {p.stem for p in folder.glob('*.py')} - {'__init__'}

    def find_spec(self, fullname, path=None, target=None):
        if fullname in self.names:
            return importlib.machinery.PathFinder.find_spec(fullname, [str(self.folder)])
        return None


class MvsCamera:
    def __init__(self, sdk_root=None):
        self.module = self.cam = self.devices = self.latest = self.thread = self.dll_directory = None
        self.connected = False
        self.latest_at = 0.0
        self.frame_id = 0
        self.sdk_frame_id = None
        self.error = ""
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.sdk_version = None
        # SDK is installed separately on each machine; no project-specific absolute path.
        program_files = Path(os.environ.get("ProgramFiles(x86)", os.environ.get("ProgramFiles", ".")))
        roots = [Path(sdk_root)] if sdk_root else []
        if os.environ.get("MVCAM_COMMON_RUNENV"):
            env_root = Path(os.environ["MVCAM_COMMON_RUNENV"])
            roots.extend([env_root, env_root.parent / "Development"])
        roots.append(program_files / "MVS" / "Development")
        wrapper = next((r / "Samples" / "Python" / "MvImport" for r in roots
                        if (r / "Samples" / "Python" / "MvImport" / "MvCameraControl_class.py").is_file()),
                       roots[-1] / "Samples" / "Python" / "MvImport")
        runtime = program_files / "Common Files" / "MVS" / "Runtime"
        dll_folder = runtime / ("Win64_x64" if struct.calcsize("P") == 8 else "Win32_i86")
        try:
            if os.name != "nt" or not (wrapper / "MvCameraControl_class.py").is_file():
                raise InputError("Không tìm thấy Python wrapper MVS của hãng cho Windows.")
            dll = dll_folder / "MvCameraControl.dll"
            with dll.open("rb") as stream:
                stream.seek(0x3C)
                offset = struct.unpack("<I", stream.read(4))[0]
                stream.seek(offset + 4)
                machine = struct.unpack("<H", stream.read(2))[0]
            if machine != (0x8664 if struct.calcsize("P") == 8 else 0x14C):
                raise InputError("DLL MVS khác architecture với Python.")
            self.dll_directory = os.add_dll_directory(str(dll_folder))
            ct.WinDLL(str(dll))
            finder = _InstalledSdkFinder(wrapper)
            sys.meta_path.insert(0, finder)
            previous = sys.dont_write_bytecode
            try:
                sys.dont_write_bytecode = True
                self.module = importlib.import_module("MvCameraControl_class")
            finally:
                sys.dont_write_bytecode = previous
                sys.meta_path.remove(finder)
            self.sdk_version = f"0x{self.module.MvCamera.MV_CC_GetSDKVersion():08X}"
            if hasattr(self.module.MvCamera, "MV_CC_Initialize"):
                self.check(self.module.MvCamera.MV_CC_Initialize(), "Khởi tạo SDK")
        except Exception as exc:
            self.module = None
            self.error = f"MVS SDK chưa khả dụng: {exc}. Vẫn có thể import PNG/JPG/BMP do MVS xuất."

    @staticmethod
    def check(code, operation):
        if code != 0:
            raise InputError(f"MVS: {operation} lỗi 0x{code & 0xFFFFFFFF:08X}; kiểm tra cáp/camera và đóng MVS nếu đang giữ camera.")

    def status(self):
        with self.lock:
            return {"available": self.module is not None, "connected": self.connected, "sdk_version": self.sdk_version,
                    "message": self.error or ("Camera đang kết nối." if self.connected else "SDK khả dụng; camera chưa kết nối.")}

    @staticmethod
    def text(array):
        return bytes(array).split(b"\0", 1)[0].decode("utf-8", errors="replace")

    def enumerate(self):
        if self.module is None:
            raise InputError(self.error)
        with self.lock:
            if self.connected:
                raise InputError("Ngắt camera trước khi liệt kê lại.")
            m = self.module
            self.devices = m.MV_CC_DEVICE_INFO_LIST()
            self.check(m.MvCamera.MV_CC_EnumDevices(m.MV_GIGE_DEVICE | m.MV_USB_DEVICE, self.devices), "Liệt kê camera")
            result = []
            for i in range(self.devices.nDeviceNum):
                info = ct.cast(self.devices.pDeviceInfo[i], ct.POINTER(m.MV_CC_DEVICE_INFO)).contents
                fields = info.SpecialInfo.stGigEInfo if info.nTLayerType == m.MV_GIGE_DEVICE else info.SpecialInfo.stUsb3VInfo
                result.append({"index": i, "name": f"{self.text(fields.chModelName)} — {self.text(fields.chSerialNumber)}"})
            return result

    def connect(self, index):
        with self.lock:
            if self.module is None:
                raise InputError(self.error)
            if self.cam is not None:
                raise InputError("Camera đã mở; hãy ngắt trước khi đổi camera.")
            if self.devices is None or index < 0 or index >= self.devices.nDeviceNum:
                raise InputError("Liệt kê và chọn camera hợp lệ trước.")
            m = self.module
            info = ct.cast(self.devices.pDeviceInfo[index], ct.POINTER(m.MV_CC_DEVICE_INFO)).contents
            self.cam = m.MvCamera()
            try:
                self.check(self.cam.MV_CC_CreateHandle(info), "Tạo handle")
                self.check(self.cam.MV_CC_OpenDevice(m.MV_ACCESS_Exclusive, 0), "Mở camera")
                if info.nTLayerType == m.MV_GIGE_DEVICE:
                    packet = self.cam.MV_CC_GetOptimalPacketSize()
                    if packet > 0:
                        self.check(self.cam.MV_CC_SetIntValue("GevSCPSPacketSize", packet), "Đặt packet size")
                self.check(self.cam.MV_CC_SetEnumValue("TriggerMode", m.MV_TRIGGER_MODE_OFF), "Đặt preview liên tục")
                self.check(self.cam.MV_CC_StartGrabbing(), "Bắt đầu stream")
            except Exception:
                self.cam.MV_CC_CloseDevice()
                self.cam.MV_CC_DestroyHandle()
                self.cam = None
                raise
            self.connected, self.error = True, ""
            self.sdk_frame_id = None
            self.stop.clear()
            self.thread = threading.Thread(target=self._grab, daemon=True, name="mvs-preview")
            self.thread.start()

    def _grab(self):
        m = self.module
        failures = 0
        while not self.stop.is_set():
            frame = m.MV_FRAME_OUT()
            try:
                code = self.cam.MV_CC_GetImageBuffer(frame, 300)
                if code != 0:
                    failures += 1
                    if failures >= 10:
                        raise InputError("Mất frame/camera ngắt kết nối; kiểm tra cáp và kết nối lại.")
                    continue
                try:
                    width, height = int(frame.stFrameInfo.nWidth), int(frame.stFrameInfo.nHeight)
                    if not 0 < width * height <= 100_000_000:
                        raise InputError("Frame camera có kích thước không hợp lệ/quá lớn.")
                    convert = m.MV_CC_PIXEL_CONVERT_PARAM_EX()
                    convert.nWidth, convert.nHeight = width, height
                    convert.pSrcData, convert.nSrcDataLen = frame.pBufAddr, frame.stFrameInfo.nFrameLen
                    convert.enSrcPixelType = frame.stFrameInfo.enPixelType
                    convert.enDstPixelType = m.PixelType_Gvsp_RGB8_Packed
                    buffer = (ct.c_ubyte * (width * height * 3))()
                    convert.pDstBuffer, convert.nDstBufferSize = buffer, len(buffer)
                    self.check(self.cam.MV_CC_ConvertPixelTypeEx(convert), "Chuyển pixel format")
                    if convert.nDstLen != len(buffer):
                        raise InputError("MVS trả buffer RGB có độ dài không đúng.")
                    rgb = np.ctypeslib.as_array(buffer).reshape(height, width, 3).copy()
                    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                    sdk_id = int(frame.stFrameInfo.nFrameNum)
                finally:
                    self.check(self.cam.MV_CC_FreeImageBuffer(frame), "Trả buffer SDK")
                failures = 0
                with self.lock:
                    self.latest, self.latest_at = bgr, time.monotonic()
                    delta = 1 if self.sdk_frame_id is None else (sdk_id - self.sdk_frame_id) & 0xFFFFFFFF
                    if delta == 0:
                        continue  # SDK repeated the same sensor frame; do not publish again.
                    self.frame_id += max(1, delta)
                    self.sdk_frame_id = sdk_id
            except Exception as exc:
                with self.lock:
                    self.error, self.connected, self.latest = str(exc), False, None
                self.stop.set()

    def frame(self):
        with self.lock:
            if not self.connected or self.latest is None or time.monotonic() - self.latest_at > 2:
                raise InputError(self.error or "Camera chưa có frame mới. Kết nối và chờ preview trước khi chụp.")
            return self.latest.copy()

    def sample(self):
        with self.lock:
            return self.frame_id, self.latest_at, self.frame()

    def disconnect(self):
        self.stop.set()
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(timeout=2)
            if self.thread.is_alive():
                raise InputError("SDK chưa dừng lấy frame; thử ngắt lại, không giải phóng buffer khi còn đang dùng.")
        with self.lock:
            errors = []
            if self.cam:
                destroyed = False
                for method, label in [("MV_CC_StopGrabbing", "Dừng stream"), ("MV_CC_CloseDevice", "Đóng camera"),
                                      ("MV_CC_DestroyHandle", "Giải phóng handle")]:
                    try:
                        self.check(getattr(self.cam, method)(), label)
                        if method == "MV_CC_DestroyHandle":
                            destroyed = True
                    except Exception as exc:
                        errors.append(str(exc))
                if destroyed:
                    self.cam = None
            self.thread, self.latest = None, None
            self.connected = False
            if errors:
                self.error = "; ".join(errors)
                raise InputError(self.error)

    def close(self):
        try:
            self.disconnect()
        finally:
            # Never finalize/unload a DLL while a live worker/handle may still use it.
            if self.cam is None:
                if self.module and hasattr(self.module.MvCamera, "MV_CC_Finalize"):
                    self.check(self.module.MvCamera.MV_CC_Finalize(), "Finalize SDK")
                self.module = None
                if self.dll_directory:
                    self.dll_directory.close()
                    self.dll_directory = None
