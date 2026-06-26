"""云台串口通信模块 — 二进制协议。

与 STM32 QGimbal 下位机通信，协议定义见下位机 Applications/Src/TransmitTask.cpp。

上位机 → 下位机（10 字节）:
    cmd(1B uint8) + yaw(4B float32 LE) + pitch(4B float32 LE) + CRC8(1B)

下位机 → 上位机（42 字节）:
    status(1B) + imu_speed_yaw(4B) + imu_speed_pitch(4B) +
    imu_angle_yaw(4B) + imu_angle_pitch(4B) +
    motor_current_yaw(4B) + motor_current_pitch(4B) +
    motor_speed_yaw(4B) + motor_speed_pitch(4B) +
    motor_angle_yaw(4B) + motor_angle_pitch(4B) + CRC8(1B)

CRC8 参数: poly=0x07, init=0x00, xor_out=0x00, 无位反转

命令码（与下位机 CmdType 枚举一致）:
    0x00 NOP              0x01 Enable           0x02 Disable
    0x03 CurrentCtrl      0x04 SpeedCtrl        0x05 AngleCtrl
    0x06 LowSpeedCtrl     0x07 StepAngleCtrl
    0xFB ResetIMU         0xFC DisableLaser     0xFD EnableLaser
    0xFE DisableStability 0xFF EnableStability
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Optional


# ------------------------------------------------------------------
# CRC8
# ------------------------------------------------------------------
def _crc8(data: bytes, poly: int = 0x07, init: int = 0x00) -> int:
    """CRC8 校验，与下位机 CRC8() 函数完全一致。"""
    crc = init
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


# ------------------------------------------------------------------
# 数据包构建 / 解析
# ------------------------------------------------------------------
def _build_packet(cmd: int, yaw: float = 0.0, pitch: float = 0.0) -> bytes:
    """构建 10 字节发送包: cmd(1B) + yaw(f32 LE) + pitch(f32 LE) + CRC8(1B)。"""
    payload = struct.pack('<Bff', cmd, yaw, pitch)
    return payload + struct.pack('<B', _crc8(payload))


def _parse_response(data: bytes) -> Optional[GimbalFeedback]:
    """解析 42 字节回复包，CRC 校验失败返回 None。"""
    if len(data) < 42:
        return None
    crc_calc = _crc8(data[:41])
    if crc_calc != data[41]:
        return None
    status = data[0]
    fields = struct.unpack('<ffffffffff', data[1:41])
    return GimbalFeedback(
        status=status,
        imu_speed_yaw=fields[0],
        imu_speed_pitch=fields[1],
        imu_angle_yaw=fields[2],
        imu_angle_pitch=fields[3],
        motor_current_yaw=fields[4],
        motor_current_pitch=fields[5],
        motor_speed_yaw=fields[6],
        motor_speed_pitch=fields[7],
        motor_angle_yaw=fields[8],
        motor_angle_pitch=fields[9],
    )


# ------------------------------------------------------------------
# 命令码
# ------------------------------------------------------------------
class Cmd:
    """与下位机 CmdType 枚举一一对应。"""
    NOP = 0x00
    ENABLE = 0x01
    DISABLE = 0x02
    CURRENT_CTRL = 0x03
    SPEED_CTRL = 0x04
    ANGLE_CTRL = 0x05
    LOW_SPEED_CTRL = 0x06
    STEP_ANGLE_CTRL = 0x07
    RESET_IMU = 0xFB
    DISABLE_LASER = 0xFC
    ENABLE_LASER = 0xFD
    DISABLE_STABILITY = 0xFE
    ENABLE_STABILITY = 0xFF


# ------------------------------------------------------------------
# 反馈数据结构
# ------------------------------------------------------------------
@dataclass
class GimbalFeedback:
    """云台下位机回传的 TxPackage 数据。

    字段顺序与下位机 TxPackage 结构体一致:
        status | imu_speed | imu_angle | motor_current | motor_speed | motor_angle
    """

    status: int
    """状态标志位（bit0: 电机使能, bit1: 自稳使能, bit2: 激光使能）"""

    imu_speed_yaw: float
    imu_speed_pitch: float
    imu_angle_yaw: float
    imu_angle_pitch: float
    motor_current_yaw: float
    motor_current_pitch: float
    motor_speed_yaw: float
    motor_speed_pitch: float
    motor_angle_yaw: float
    motor_angle_pitch: float

    @property
    def enabled(self) -> bool:
        return bool(self.status & 0x01)

    @property
    def stability_enabled(self) -> bool:
        return bool(self.status & 0x02)

    @property
    def laser_enabled(self) -> bool:
        return bool(self.status & 0x04)

    def __repr__(self) -> str:
        return (
            f"GimbalFeedback(enabled={self.enabled}, stability={self.stability_enabled}, "
            f"laser={self.laser_enabled}, "
            f"imu_spd=({self.imu_speed_yaw:.2f},{self.imu_speed_pitch:.2f})rpm "
            f"imu_ang=({self.imu_angle_yaw:.3f},{self.imu_angle_pitch:.3f})rad)"
        )


# ------------------------------------------------------------------
# 串口通信封装
# ------------------------------------------------------------------
@dataclass
class GimbalSerial:
    """云台串口通信封装（二进制协议）。

    Usage::

        gimbal = GimbalSerial(port="/dev/ttyAMA0")
        gimbal.open()
        gimbal.enable()

        # 速度控制（仅发送，不等待回复，适合高频调用）
        gimbal.speed_ctrl(yaw=10.0, pitch=0.0)

        # 查询状态（发送 NOP + 读取回复）
        fb = gimbal.query()
        if fb:
            print(fb.imu_angle_yaw)
        gimbal.close()
    """

    port: Optional[str] = None
    baudrate: int = 115200

    _ser: object = field(default=None, init=False, repr=False)

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------
    def open(self) -> None:
        if self.port is None:
            self._ser = None
            return
        import serial
        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=0.1,
            write_timeout=0,
        )

    def close(self) -> None:
        ser = self._ser
        self._ser = None
        if ser is not None and callable(getattr(ser, "close", None)):
            ser.close()

    # ------------------------------------------------------------------
    # 底层收发
    # ------------------------------------------------------------------
    def _send_cmd(self, cmd: int, yaw: float = 0.0, pitch: float = 0.0) -> None:
        """发送命令包，不等待回复。"""
        if self._ser is None:
            return
        pkt = _build_packet(cmd, yaw, pitch)
        self._ser.write(pkt)
        self._ser.flush()

    def _recv_response(self) -> Optional[GimbalFeedback]:
        """读取 42 字节回复包并解析。"""
        if self._ser is None:
            return None
        try:
            resp = self._ser.read(42)
        except Exception:
            return None
        if not resp:
            return None
        return _parse_response(resp)

    def _tx_rx(self, cmd: int, yaw: float = 0.0, pitch: float = 0.0) -> Optional[GimbalFeedback]:
        """发送命令并等待回复。"""
        self._send_cmd(cmd, yaw, pitch)
        return self._recv_response()

    # ------------------------------------------------------------------
    # 开关类命令（同步：发送 + 读取回复）
    # ------------------------------------------------------------------
    def enable(self) -> Optional[GimbalFeedback]:
        return self._tx_rx(Cmd.ENABLE)

    def disable(self) -> Optional[GimbalFeedback]:
        return self._tx_rx(Cmd.DISABLE)

    def enable_stability(self) -> Optional[GimbalFeedback]:
        return self._tx_rx(Cmd.ENABLE_STABILITY)

    def disable_stability(self) -> Optional[GimbalFeedback]:
        return self._tx_rx(Cmd.DISABLE_STABILITY)

    def enable_laser(self) -> Optional[GimbalFeedback]:
        return self._tx_rx(Cmd.ENABLE_LASER)

    def disable_laser(self) -> Optional[GimbalFeedback]:
        return self._tx_rx(Cmd.DISABLE_LASER)

    def reset_imu(self) -> Optional[GimbalFeedback]:
        return self._tx_rx(Cmd.RESET_IMU)

    # ------------------------------------------------------------------
    # 控制类命令
    # ------------------------------------------------------------------
    def angle_ctrl(self, yaw: float, pitch: float) -> Optional[GimbalFeedback]:
        """角度控制（rad），同步返回反馈。"""
        return self._tx_rx(Cmd.ANGLE_CTRL, yaw, pitch)

    def speed_ctrl(self, yaw: float, pitch: float) -> None:
        """速度控制（RPM），仅发送不等待回复。

        main.py 中 30fps 高频调用，为避免阻塞主循环，只发送不读取。
        每个命令 STM32 都会回复 42 字节，串口驱动缓冲区会自动接收，
        如需获取反馈可调用 query()。
        """
        self._send_cmd(Cmd.SPEED_CTRL, yaw, pitch)

    def current_ctrl(self, yaw: float, pitch: float) -> Optional[GimbalFeedback]:
        """电流控制（A），同步返回反馈。"""
        return self._tx_rx(Cmd.CURRENT_CTRL, yaw, pitch)

    def low_speed_ctrl(self, yaw: float, pitch: float) -> Optional[GimbalFeedback]:
        """低速控制（RPM），同步返回反馈。"""
        return self._tx_rx(Cmd.LOW_SPEED_CTRL, yaw, pitch)

    def step_angle_ctrl(self, yaw: float, pitch: float) -> Optional[GimbalFeedback]:
        """步进角度控制（rad），同步返回反馈。"""
        return self._tx_rx(Cmd.STEP_ANGLE_CTRL, yaw, pitch)

    # ------------------------------------------------------------------
    # 查询
    # ------------------------------------------------------------------
    def query(self) -> Optional[GimbalFeedback]:
        """发送 NOP 命令并读取反馈（与 nop 等价）。"""
        return self._tx_rx(Cmd.NOP)

    def nop(self) -> Optional[GimbalFeedback]:
        """发送 NOP 命令并读取反馈。"""
        return self.query()


# ------------------------------------------------------------------
# 命令行测试入口
# ------------------------------------------------------------------
if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='QGimbal 上位机串口测试（二进制协议）')
    parser.add_argument('port', help='串口设备，如 /dev/ttyUSB0')
    parser.add_argument('cmd', nargs='?', help='指令: enable, disable, nop, speed Y P, angle Y P, ...')
    parser.add_argument('args', nargs='*', type=float, help='yaw pitch 参数')
    parser.add_argument('-b', '--baud', type=int, default=115200, help='波特率（默认 115200）')
    args = parser.parse_args()

    g = GimbalSerial(port=args.port, baudrate=args.baud)
    try:
        g.open()
    except Exception as e:
        print(f"打开串口失败: {e}")
        import sys
        sys.exit(1)

    # 命令名 → (Cmd 码, 是否需要 yaw/pitch 参数)
    CMD_MAP = {
        'nop':              (Cmd.NOP,              False),
        'enable':           (Cmd.ENABLE,           False),
        'disable':          (Cmd.DISABLE,          False),
        'current':          (Cmd.CURRENT_CTRL,     True),
        'speed':            (Cmd.SPEED_CTRL,       True),
        'angle':            (Cmd.ANGLE_CTRL,       True),
        'lowspeed':         (Cmd.LOW_SPEED_CTRL,   True),
        'step':             (Cmd.STEP_ANGLE_CTRL,  True),
        'reset_imu':        (Cmd.RESET_IMU,        False),
        'laser_on':         (Cmd.ENABLE_LASER,     False),
        'laser_off':        (Cmd.DISABLE_LASER,    False),
        'stability_on':     (Cmd.ENABLE_STABILITY, False),
        'stability_off':    (Cmd.DISABLE_STABILITY,False),
    }

    if args.cmd is None:
        # 交互模式
        print("QGimbal 上位机 — 输入 help 查看指令")
        while True:
            try:
                line = input("> ").strip()
                if not line:
                    continue
                parts = line.split()
                name = parts[0].lower()
                vals = [float(x) for x in parts[1:3]] if len(parts) > 1 else []

                if name == 'help':
                    print("  开关: enable disable stability_on/off laser_on/off reset_imu")
                    print("  查询: nop")
                    print("  控制: speed Y P  /  angle Y P  /  current Y P  /  step Y P")
                    print("  quit  退出")
                elif name == 'quit':
                    break
                elif name in CMD_MAP:
                    code, need_params = CMD_MAP[name]
                    if need_params:
                        y, p = (vals + [0.0, 0.0])[:2]
                        fb = g._tx_rx(code, y, p)
                    else:
                        fb = g._tx_rx(code)
                    print(f"  {fb}" if fb else "  无响应")
                else:
                    print(f"  未知指令: {name}")
            except (EOFError, KeyboardInterrupt):
                break
            except Exception as e:
                print(f"  错误: {e}")
    else:
        # 单条指令
        name = args.cmd.lower()
        vals = args.args[:2] if args.args else []
        if name not in CMD_MAP:
            print(f"未知指令: {name}")
            g.close()
            import sys
            sys.exit(1)
        code, need_params = CMD_MAP[name]
        if need_params:
            y, p = (vals + [0.0, 0.0])[:2]
            fb = g._tx_rx(code, y, p)
        else:
            fb = g._tx_rx(code)
        print(fb if fb else "无响应")

    g.close()
