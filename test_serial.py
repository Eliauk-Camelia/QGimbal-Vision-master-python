"""云台串口通信测试脚本（二进制协议）。
在树莓派上运行: python test_serial.py --port /dev/ttyAMA0

协议: 10 字节发送, 42 字节回复, 115200 8N1, CRC8(poly=0x07)
对应下位机 Applications/Src/TransmitTask.cpp
"""

import argparse
import time
import sys

sys.path.insert(0, '.')
from control.serial_stub import GimbalSerial


def parse_args():
    p = argparse.ArgumentParser(description="测试 STM32 云台串口通信（二进制协议）")
    p.add_argument('--port', type=str, required=True, help='串口端口，如 /dev/ttyAMA0')
    p.add_argument('--baud', type=int, default=115200)
    return p.parse_args()


def main():
    args = parse_args()
    gimbal = GimbalSerial(port=args.port, baudrate=args.baud)
    print(f"正在打开串口 {args.port}...")
    try:
        gimbal.open()
    except Exception as e:
        print(f"打开串口失败: {e}")
        return
    print("串口已打开\n")

    try:
        # ---- 测试 1: NOP 查询 ----
        print("=" * 50)
        print("测试 1: NOP 查询状态 (5次)")
        print("=" * 50)
        for i in range(5):
            fb = gimbal.query()
            if fb:
                print(f"  [{i+1}] {fb}")
            else:
                print(f"  [{i+1}] 无响应 — 检查接线 / STM32 是否使能（IMU 正常初始化后云台才使能）")
            time.sleep(0.1)

        # ---- 测试 2: 使能/失能 ----
        print("\n" + "=" * 50)
        print("测试 2: 使能 → 查询 → 失能 → 查询")
        print("=" * 50)
        fb = gimbal.enable()
        print(f"  使能回传: {fb}")
        time.sleep(0.1)
        fb = gimbal.query()
        print(f"  使能后查询: {fb}")

        time.sleep(0.5)

        fb = gimbal.disable()
        print(f"  失能回传: {fb}")
        time.sleep(0.1)
        fb = gimbal.query()
        print(f"  失能后查询: {fb}")

        # ---- 测试 3: 速度控制 (慎用，电机会转) ----
        print("\n" + "=" * 50)
        print("测试 3: 速度控制 SPD 10 0 → 立刻停止")
        print("  （注意: speed_ctrl 为 fire-and-forget，不读回复）")
        print("=" * 50)
        gimbal.enable()
        time.sleep(0.05)
        gimbal.speed_ctrl(yaw=10.0, pitch=0.0)
        time.sleep(0.5)
        fb = gimbal.query()
        print(f"  SPD(10,0) 后查询: {fb}")

        gimbal.speed_ctrl(yaw=0.0, pitch=0.0)
        time.sleep(0.1)
        gimbal.disable()

        # ---- 测试 4: IMU 连续读取 ----
        print("\n" + "=" * 50)
        print("测试 4: 连续读取 IMU 数据 (10次)")
        print("=" * 50)
        for i in range(10):
            fb = gimbal.query()
            if fb:
                print(
                    f"  [{i+1:2d}] en={fb.enabled} stb={fb.stability_enabled} "
                    f"imu_s=({fb.imu_speed_yaw:7.2f},{fb.imu_speed_pitch:7.2f}) "
                    f"imu_a=({fb.imu_angle_yaw:7.3f},{fb.imu_angle_pitch:7.3f})"
                )
            else:
                print(f"  [{i+1:2d}] 无响应")
            time.sleep(0.05)

        print("\n所有测试完成 ✅")

    except KeyboardInterrupt:
        print("\n中断退出")
    finally:
        gimbal.close()
        print("串口已关闭")


if __name__ == '__main__':
    main()
