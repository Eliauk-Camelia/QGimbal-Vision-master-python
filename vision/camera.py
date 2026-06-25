"""摄像头管理：枚举、打开、配置、读取、释放。"""

import cv2
import numpy as np
from typing import Optional, List, Tuple


def list_cameras(max_index: int = 8) -> List[Tuple[int, str]]:
    """枚举系统中可用的摄像头，返回 [(索引, 名称), ...]。

    树莓派 USB 摄像头提示：
    - USB 摄像头通常出现在 /dev/video0, /dev/video1...
    - 若 CSI 摄像头也占用一个索引，USB 摄像头索引可能为 1
    - 可以用 `ls /dev/video*` 在终端确认设备节点
    """
    available = []
    for i in range(max_index):
        cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
        if cap.isOpened():
            # 尝试读取一帧确认真的可用（部分设备能打开但无法读帧）
            ret, _ = cap.read()
            if ret:
                backend = cap.getBackendName()
                w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                fps = cap.get(cv2.CAP_PROP_FPS)
                name = f"{backend} {int(w)}x{int(h)}@{fps:.0f}fps"
                available.append((i, name))
            else:
                cap.release()
                continue
        cap.release()
    return available


class CameraManager:
    """封装 OpenCV VideoCapture 的打开、参数配置和释放。

    USB 摄像头注意事项：
    - 默认使用 V4L2 后端（Linux 上 USB UVC 设备的标准接口）
    - FPS 默认 30（大多数 USB 摄像头实用上限）
    - 打开后会验证实际生效的宽/高/FPS
    """

    # 多数 USB 摄像头的实际帧率上限
    DEFAULT_FPS = 30

    def __init__(
        self,
        index: int = 0,
        width: int = 640,
        height: int = 480,
        fps: int = DEFAULT_FPS,
        backend: int = cv2.CAP_V4L2,
    ) -> None:
        self._index = index
        self._width = width
        self._height = height
        self._fps = fps
        self._backend = backend
        self._cap: Optional[cv2.VideoCapture] = None

    @property
    def index(self) -> int:
        return self._index

    @property
    def actual_width(self) -> int:
        return int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)) if self._cap is not None else 0

    @property
    def actual_height(self) -> int:
        return int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) if self._cap is not None else 0

    @property
    def actual_fps(self) -> float:
        return self._cap.get(cv2.CAP_PROP_FPS) if self._cap is not None else 0.0

    def open(self) -> None:
        """打开摄像头并设置参数。失败时抛出 RuntimeError 并附带诊断信息。"""
        # 先尝试 V4L2 后端（USB 摄像头在 Linux 上的标准后端）
        self._cap = cv2.VideoCapture(self._index, self._backend)
        if not self._cap.isOpened():
            # 回退到默认后端
            self._cap = cv2.VideoCapture(self._index)
        if not self._cap.isOpened():
            raise RuntimeError(
                f"无法打开摄像头索引 {self._index}。\n"
                f"  USB 摄像头排查：\n"
                f"  1. 检查设备节点: ls /dev/video*\n"
                f"  2. 确认用户有权限: groups | grep video\n"
                f"  3. 尝试其他索引: python main.py --camera 1\n"
                f"  4. 枚举所有摄像头: python main.py --list-cameras"
            )

        # 设置期望参数（USB 摄像头可能不支持，OpenCV 会静默回退）
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        self._cap.set(cv2.CAP_PROP_FPS, self._fps)

        # 读取实际生效的参数
        real_w = self.actual_width
        real_h = self.actual_height
        real_fps = self.actual_fps

        print(
            f"摄像头 {self._index} 已打开 ({self._cap.getBackendName()}): "
            f"请求 {self._width}x{self._height}@{self._fps}fps → "
            f"实际 {real_w}x{real_h}@{real_fps:.1f}fps"
        )

    def read(self) -> Optional[np.ndarray]:
        """读取一帧，失败时返回 None。"""
        if self._cap is None:
            raise RuntimeError("摄像头未打开，请先调用 open()。")
        ret, frame = self._cap.read()
        if not ret or frame is None:
            return None
        return frame

    def release(self) -> None:
        """释放摄像头资源。"""
        if self._cap is not None:
            self._cap.release()
            self._cap = None
