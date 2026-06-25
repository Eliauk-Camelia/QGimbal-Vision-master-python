"""云台串口通信模块。

定义与 STM32 云台下位机通信的：
- 命令码（CmdType）
- CRC8 校验
- 反馈数据解析（GimbalFeedback）
- 串口收发封装（GimbalSerial）

数据包格式（小端序）：
  上位机 → 下位机：  cmd(uint8) + yaw(float32) + pitch(float32) + crc8(uint8)  = 10 字节
  下位机 → 上位机：  status(uint8) + 10×float32 + crc8(uint8)                 = 42 字节
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Optional


class CmdType:
    """下位机命令码枚举，通过串口发送给 STM32 云台。

    控制类命令（yaw/pitch 参数生效）:
        NOP            - 空操作，下位机收到后返回当前状态
        CurrentCtrl    - 电流环控制
        SpeedCtrl      - 速度环控制
        AngleCtrl      - 角度环控制
        LowSpeedCtrl   - 低速模式速度控制
        StepAngleCtrl  - 步进角度控制

    开关类命令（yaw/pitch 参数无效）:
        Enable          - 使能云台电机
        Disable         - 失能云台电机
        EnableStability - 开启自稳
        DisableStability- 关闭自稳
        EnableLaser     - 打开激光
        DisableLaser    - 关闭激光
        ResetIMU        - 复位 IMU
    """

    NOP = 0x00
    Enable = 0x01
    Disable = 0x02
    CurrentCtrl = 0x03
    SpeedCtrl = 0x04
    AngleCtrl = 0x05
    LowSpeedCtrl = 0x06
    StepAngleCtrl = 0x07
    EnableStability = 0xFF
    DisableStability = 0xFE
    EnableLaser = 0xFD
    DisableLaser = 0xFC
    ResetIMU = 0xFB


def crc8(data: bytes) -> int:
    """计算 8 位 CRC 校验值（多项式 0x07）。

    用于数据包完整性校验，下位机使用相同算法验证。
    """
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


@dataclass
class GimbalFeedback:
    """云台下位机回传的反馈数据。

    每帧 42 字节，包含 IMU 和电机两套传感器数据：
        - IMU 数据：角速度、角度（用于自稳闭环）
        - 电机数据：电流、速度、角度（用于电机控制闭环）
        - status 字段低 3 位分别表示 enabled / stability / laser 状态
    """

    status: int
    """状态标志位（bit0: 电机使能, bit1: 自稳使能, bit2: 激光使能）"""

    imu_speed_yaw: float
    """IMU 偏航角速度（度/秒）"""
    imu_speed_pitch: float
    """IMU 俯仰角速度（度/秒）"""
    imu_angle_yaw: float
    """IMU 偏航角度（度）"""
    imu_angle_pitch: float
    """IMU 俯仰角度（度）"""

    motor_current_yaw: float
    """偏航电机电流（A）"""
    motor_current_pitch: float
    """俯仰电机电流（A）"""
    motor_speed_yaw: float
    """偏航电机转速（RPM）"""
    motor_speed_pitch: float
    """俯仰电机转速（RPM）"""
    motor_angle_yaw: float
    """偏航电机编码器角度（度）"""
    motor_angle_pitch: float
    """俯仰电机编码器角度（度）"""

    @property
    def enabled(self) -> bool:
        """电机是否已使能（status bit0）。"""
        return bool(self.status & 0x01)

    @property
    def stability_enabled(self) -> bool:
        """自稳是否已开启（status bit1）。"""
        return bool(self.status & 0x02)

    @property
    def laser_enabled(self) -> bool:
        """激光是否已打开（status bit2）。"""
        return bool(self.status & 0x04)

    def __repr__(self) -> str:
        return (f"GimbalFeedback(enabled={self.enabled}, stability={self.stability_enabled}, "
                f"laser={self.laser_enabled}, "
                f"imu_angle=({self.imu_angle_yaw:.3f}, {self.imu_angle_pitch:.3f}), "
                f"imu_speed=({self.imu_speed_yaw:.1f}, {self.imu_speed_pitch:.1f}))")


@dataclass
class GimbalSerial:
    """云台串口通信封装。

    提供与 STM32 下位机的双向通信：
        - 发送：各种控制命令（使能、速度、角度等）
        - 接收：查询反馈数据（IMU 姿态、电机状态）

    Usage::

        gimbal = GimbalSerial(port="/dev/ttyUSB0")
        gimbal.open()
        gimbal.enable()
        gimbal.speed_ctrl(yaw=10.0, pitch=0.0)

        feedback = gimbal.query()
        if feedback:
            print(feedback.imu_angle_yaw)
        gimbal.close()
    """

    port: Optional[str] = None
    """串口端口号，如 ``/dev/ttyUSB0`` (Linux) 或 ``COM3`` (Windows)。
       为 None 时不打开串口，所有操作静默跳过。"""

    baudrate: int = 115200
    """串口波特率，默认 115200。"""

    _ser: object = field(default=None, init=False, repr=False)
    """内部的 pyserial 实例，类型为 ``serial.Serial``。
       未打开或 port 为 None 时保持为 None。"""

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    def open(self) -> None:
        """打开串口。

        若 ``port`` 为 None，则跳过（允许无硬件运行）。
        pyserial 采用延迟导入，未安装时不影响其他模块加载。
        """
        if self.port is None:
            self._ser = None
            return
        import serial  # type: ignore
        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=0.1,        # 读超时 100ms，避免 nop() 阻塞主循环
            write_timeout=0,
        )

    def close(self) -> None:
        """关闭串口，安全释放资源。"""
        ser = self._ser
        self._ser = None
        if ser is not None and callable(getattr(ser, "close", None)):
            ser.close()

    # ------------------------------------------------------------------
    # 底层收发
    # ------------------------------------------------------------------

    def _send(self, cmd: int, yaw: float = 0.0, pitch: float = 0.0) -> None:
        """发送一帧数据包（10 字节）。

        Args:
            cmd: 命令码，见 CmdType
            yaw: 偏航参数（角度/速度/电流，取决于命令类型）
            pitch: 俯仰参数

        数据包格式（小端序）::

            [cmd: u8] [yaw: f32] [pitch: f32] [crc8: u8]
        """
        if self._ser is None:
            return
        payload = struct.pack("<Bff", cmd, float(yaw), float(pitch))
        pkt = payload + struct.pack("<B", crc8(payload))
        self._ser.write(pkt)

    def _recv(self) -> Optional[GimbalFeedback]:
        """接收下位机回传的反馈数据包（42 字节）。

        Returns:
            解析成功返回 GimbalFeedback，否则返回 None。
            None 的情况包括：串口未打开、数据不足、CRC 校验失败。
        """
        if self._ser is None:
            return None
        data = self._ser.read(42)
        # 长度校验 + CRC 校验，任一失败丢弃本帧
        if len(data) != 42 or crc8(data[:-1]) != data[-1]:
            return None
        vals = struct.unpack("<B5ff", data[:-1])
        return GimbalFeedback(
            status=vals[0],
            imu_speed_yaw=vals[1], imu_speed_pitch=vals[2],
            imu_angle_yaw=vals[3], imu_angle_pitch=vals[4],
            motor_current_yaw=vals[5], motor_current_pitch=vals[6],
            motor_speed_yaw=vals[7], motor_speed_pitch=vals[8],
            motor_angle_yaw=vals[9], motor_angle_pitch=vals[10],
        )

    # ------------------------------------------------------------------
    # 开关类命令（无需 yaw/pitch 参数）
    # ------------------------------------------------------------------

    def enable(self) -> None:
        """使能云台电机。"""
        self._send(CmdType.Enable)

    def disable(self) -> None:
        """失能云台电机（电机断电，可自由转动）。"""
        self._send(CmdType.Disable)

    def enable_stability(self) -> None:
        """开启 IMU 自稳（下位机自动保持姿态）。"""
        self._send(CmdType.EnableStability)

    def disable_stability(self) -> None:
        """关闭 IMU 自稳。"""
        self._send(CmdType.DisableStability)

    def enable_laser(self) -> None:
        """打开激光。"""
        self._send(CmdType.EnableLaser)

    def disable_laser(self) -> None:
        """关闭激光。"""
        self._send(CmdType.DisableLaser)

    def reset_imu(self) -> None:
        """复位 IMU（将当前姿态归零）。"""
        self._send(CmdType.ResetIMU)

    # ------------------------------------------------------------------
    # 控制类命令（yaw/pitch 参数生效）
    # ------------------------------------------------------------------

    def angle_ctrl(self, yaw: float, pitch: float) -> None:
        """角度环控制。

        Args:
            yaw: 目标偏航角度（度）
            pitch: 目标俯仰角度（度）
        """
        self._send(CmdType.AngleCtrl, yaw, pitch)

    def speed_ctrl(self, yaw: float, pitch: float) -> None:
        """速度环控制。

        Args:
            yaw: 目标偏航角速度（RPM）
            pitch: 目标俯仰角速度（RPM）
        """
        self._send(CmdType.SpeedCtrl, yaw, pitch)

    def current_ctrl(self, yaw: float, pitch: float) -> None:
        """电流环控制（力矩模式）。

        Args:
            yaw: 偏航电机目标电流（A）
            pitch: 俯仰电机目标电流（A）
        """
        self._send(CmdType.CurrentCtrl, yaw, pitch)

    def low_speed_ctrl(self, yaw: float, pitch: float) -> None:
        """低速模式速度控制。

        Args:
            yaw: 偏航目标低速（RPM）
            pitch: 俯仰目标低速（RPM）
        """
        self._send(CmdType.LowSpeedCtrl, yaw, pitch)

    def step_angle_ctrl(self, yaw: float, pitch: float) -> None:
        """步进角度控制（相对角度增量）。

        Args:
            yaw: 偏航增量角度（度）
            pitch: 俯仰增量角度（度）
        """
        self._send(CmdType.StepAngleCtrl, yaw, pitch)

    # ------------------------------------------------------------------
    # 查询
    # ------------------------------------------------------------------

    def nop(self) -> Optional[GimbalFeedback]:
        """发送空操作命令并等待反馈。

        下位机收到 NOP 后返回当前完整状态，这是主要的查询方式。

        Returns:
            解析成功返回 GimbalFeedback，失败返回 None。
        """
        self._send(CmdType.NOP)
        return self._recv()

    def query(self) -> Optional[GimbalFeedback]:
        """查询当前状态（nop 的别名，语义更清晰）。"""
        return self.nop()