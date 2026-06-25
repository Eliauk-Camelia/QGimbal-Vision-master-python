"""FPS 计数器，基于指数移动平均 (EMA)。"""


class FpsCounter:
    """使用 EMA 平滑计算的帧率计数器。"""

    def __init__(self, alpha: float = 0.98) -> None:
        """alpha: 平滑系数，越大越平滑，越小越灵敏。"""
        self._alpha = alpha
        self._value = 0.0

    @property
    def value(self) -> float:
        """当前平滑后的 FPS 值。"""
        return self._value

    def update(self, dt: float) -> float:
        """根据帧间隔 dt（秒）更新并返回平滑 FPS。"""
        if dt <= 0:
            return self._value
        inst = 1.0 / dt
        if self._value <= 0:
            self._value = inst
        else:
            self._value = self._alpha * self._value + (1 - self._alpha) * inst
        return self._value
