# 摄像头读取并显示画面
# 使用: python main.py --camera 0

"""
QGimbal 视觉跟踪控制 - 摄像头矩形检测 + PID 云台控制。

参数：
  --camera        摄像头索引（默认 0）
  --display       是否显示图形化窗口（0/1，默认 1）
                 - 1：显示图像，并叠加矩形与 FPS
                 - 0：不显示窗口，在终端输出 FPS + 检测到的矩形中心点坐标/面积
  --print-interval 终端输出间隔秒数（仅 --display 0 时生效，默认 0.05）

GUI 模式按 'q' 或 ESC 退出；无窗口模式请按 Ctrl+C 退出。
"""

import argparse     # 命令行参数解析
import time         # 时间戳
import sys          # 退出

import cv2

from vision.camera import CameraManager, list_cameras
from vision.fps import FpsCounter
from vision.rect_detect import detect_rectangles, draw_detected_rect

from control.config import ControlConfig
from control.serial_stub import GimbalSerial
from control.tracker_control import GimbalTracker

DEFAULT_CAMERA = 0  # 摄像头索引（默认 0）
DEFAULT_WIDTH = 640  # 期望宽度
DEFAULT_HEIGHT = 480  # 期望高度
DEFAULT_FPS = 30  # 期望帧率（USB 摄像头通常最高 30fps）
DEFAULT_DISPLAY = 1
DEFAULT_PRINT_INTERVAL = 0.05

# 控制默认参数（可通过命令行覆盖）
DEFAULT_CONTROL_ENABLED = 1
DEFAULT_MAX_RPM = 20.0
DEFAULT_DEADBAND_PX = 0.0
DEFAULT_LOST_TIMEOUT_S = 0.4


def parse_args():
    """解析命令行参数。"""
    p = argparse.ArgumentParser(description="QGimbal 视觉跟踪控制 - 摄像头矩形检测 + PID 云台控制")
    p.add_argument('--camera', type=int, default=DEFAULT_CAMERA, help=f'摄像头索引（默认 {DEFAULT_CAMERA}）')
    p.add_argument('--display', type=int, choices=[0, 1], default=DEFAULT_DISPLAY,
                   help=f'是否显示图形化窗口（0/1，默认 {DEFAULT_DISPLAY}）')
    p.add_argument('--print-interval', type=float, default=DEFAULT_PRINT_INTERVAL,
                   help=f'终端输出间隔秒数（仅 --display 0 生效，默认 {DEFAULT_PRINT_INTERVAL}）')

    # 控制相关
    p.add_argument('--control', type=int, choices=[0, 1], default=DEFAULT_CONTROL_ENABLED,
                   help=f'是否启用 PID 控制输出（0/1，默认 {DEFAULT_CONTROL_ENABLED}）')
    p.add_argument('--max-rpm', type=float, default=DEFAULT_MAX_RPM,
                   help=f'最大转速输出（RPM，默认 {DEFAULT_MAX_RPM}）')
    p.add_argument('--deadband-px', type=float, default=DEFAULT_DEADBAND_PX,
                   help=f'像素死区（默认 {DEFAULT_DEADBAND_PX}）')
    p.add_argument('--lost-timeout', type=float, default=DEFAULT_LOST_TIMEOUT_S,
                   help=f'丢目标超时后复位控制器的时间（秒，默认 {DEFAULT_LOST_TIMEOUT_S}）')

    # 串口相关（协议在 control/serial_stub.py 内实现）
    p.add_argument('--serial-port', type=str, default=None, help='串口端口号，例如 /dev/ttyUSB0 或 COM3；不填则不发送')
    p.add_argument('--serial-baud', type=int, default=115200, help='串口波特率（默认 115200）')

    # 调试
    p.add_argument('--list-cameras', action='store_true', help='枚举所有可用摄像头并退出')

    return p.parse_args()


def main():
    """主函数：初始化摄像头和控制参数，开始跟踪和控制流程。"""
    args = parse_args()         #  

    # --list-cameras：枚举摄像头后退出
    if args.list_cameras:
        cameras = list_cameras()
        if not cameras:
            print("未检测到可用摄像头。")
            print("  排查：ls /dev/video*  查看设备节点")
            print("         groups | grep video  确认用户权限")
        else:
            print("可用的摄像头：")
            for idx, name in cameras:
                print(f"  /dev/video{idx}  →  {name}")
        sys.exit(0)

    camera = CameraManager(index=args.camera, width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT, fps=DEFAULT_FPS)
    try:
        camera.open()
    except RuntimeError as e:
        print(e)
        sys.exit(2)

    display = bool(args.display)

    # 控制器初始化
    ctrl_cfg = ControlConfig(
        enabled=bool(args.control),
        deadband_px=float(args.deadband_px),
        lost_timeout_s=float(args.lost_timeout),
        max_rpm_yaw=float(args.max_rpm),
        max_rpm_pitch=float(args.max_rpm),
    )
    tracker = GimbalTracker(ctrl_cfg)
    gimbal = GimbalSerial(port=args.serial_port, baudrate=int(args.serial_baud))
    gimbal.open()

    win_name = f"Camera {args.camera}"
    if display:
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)

    # 无窗口模式：终端输出节流
    last_print = 0.0
    prev_time = time.time()
    fps_counter = FpsCounter(alpha=0.98)

    try:
        while True:
            frame = camera.read()
            if frame is None:
                print("无法从摄像头读取到帧，正在重试...")
                time.sleep(0.1)
                continue

            frame = cv2.flip(frame, -1)

            # 对每帧执行矩形检测
            rects = detect_rectangles(frame, min_area_ratio=0.005, max_area_ratio=0.5, angle_tol=25.0)
            best = rects[0] if rects else None

            # 计算 FPS（指数移动平均以平滑显示）
            now = time.time()
            dt = now - prev_time
            prev_time = now
            fps = fps_counter.update(dt)

            # PID 控制：将目标中心追踪到屏幕中心，输出 yaw/pitch rpm
            h, w = frame.shape[:2]
            target_center = best.center if best is not None else None
            ret, ctrl_out = tracker.update(frame_w=w, frame_h=h, target_center=target_center, dt=max(dt, 1e-6), now=now)
            if ret:
                gimbal.speed_ctrl(ctrl_out.yaw_rpm, ctrl_out.pitch_rpm)

            if display:
                if best is not None:
                    draw_detected_rect(frame, best)

                # 画面中心点
                cv2.drawMarker(frame, (w // 2, h // 2), (255, 0, 0), markerType=cv2.MARKER_CROSS, markerSize=18, thickness=2)

                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                cv2.putText(
                    frame,
                    f"err(px)=({ctrl_out.err_x_px:.0f},{ctrl_out.err_y_px:.0f}) rpm=({ctrl_out.yaw_rpm:.1f},{ctrl_out.pitch_rpm:.1f})",
                    (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2,
                )

                cv2.imshow(win_name, frame)
                key = cv2.waitKey(1) & 0xFF
                # 按 'q' 或 ESC 退出
                if key == ord('q') or key == 27:
                    break
            else:
                # 无窗口：终端输出 FPS + 检测结果（按间隔打印，避免刷屏）
                if args.print_interval <= 0 or (now - last_print) >= args.print_interval:
                    last_print = now
                    if best is None:
                        print(f"fps={fps:.1f} rect=none rpm=({ctrl_out.yaw_rpm:.1f},{ctrl_out.pitch_rpm:.1f})")
                    else:
                        cx, cy = best.center
                        area = best.area
                        print(
                            f"fps={fps:.1f} cx={cx:.1f} cy={cy:.1f} area={area:.0f} "
                            f"err=({ctrl_out.err_x_px:.0f},{ctrl_out.err_y_px:.0f}) rpm=({ctrl_out.yaw_rpm:.1f},{ctrl_out.pitch_rpm:.1f})"
                        )

    except KeyboardInterrupt:
        print('\n收到中断，退出...')
    finally:
        camera.release()
        gimbal.close()
        if display:
            cv2.destroyAllWindows()


# 树莓派IP :  eualik  10.194.157.81 
if __name__ == '__main__':
    main()
