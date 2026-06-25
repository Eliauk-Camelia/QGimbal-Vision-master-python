"""Vision modules for this project.

This package contains small, reusable computer vision utilities used by `main.py`.
"""

from .camera import CameraManager, list_cameras
from .fps import FpsCounter
from .rect_detect import DetectedRect, detect_rectangles, draw_detected_rect

__all__ = [
    "CameraManager",
    "DetectedRect",
    "FpsCounter",
    "detect_rectangles",
    "draw_detected_rect",
    "list_cameras",
]
