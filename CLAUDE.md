# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 常用命令

```bash
# 安装依赖
pip install -r requirements.txt

# 枚举可用摄像头
python main.py --list-cameras

# GUI 模式（显示检测窗口，按 q/ESC 退出）
python main.py --camera 0

# 无窗口模式（终端输出 FPS + 检测结果，Ctrl+C 退出）
python main.py --camera 0 --display 0 --print-interval 0.1

# 启用 PID 控制输出（配合云台硬件）
python main.py --camera 0 --control 1 --max-rpm 120 --deadband-px 6

# 连接串口发送控制命令（树莓派 GPIO 排针）
python main.py --camera 0 --serial-port /dev/ttyAMA0 --serial-baud 115200

# 串口通信测试（需要 STM32 已烧录新协议固件且 IMU 正常）
python test_serial.py --port /dev/ttyAMA0

# 运行单元测试
pytest
```

## 架构概览

项目分为两个独立包，`main.py` 串联完整管线：

```
CameraManager → [BGR frame] → detect_rectangles() → best DetectedRect
     ↓                                                      ↓
  FpsCounter                                          GimbalTracker
                                                       (PID × 2)
                                                          ↓
                                               GimbalSerial → STM32
```

### `vision/` — 摄像头 + 视觉检测

- **`camera.py`**: `CameraManager` 封装 V4L2 打开/读取/释放，`list_cameras()` 枚举 `/dev/video*` 节点并验证可读帧。
- **`rect_detect.py`**: 核心管线：灰度 → 高斯模糊 → OTSU 二值化 → 形态学开运算 → Canny 边缘 → 轮廓多边形逼近 → 角度验证（四角接近 90°）→ 返回按面积降序的 `DetectedRect` 列表。`detect_rectangles()` 内完成全部处理，输入 BGR frame，输出 `List[DetectedRect]`。
- **`fps.py`**: `FpsCounter` 用 EMA（α=0.98）平滑帧率，输入帧间隔 dt（秒），调用 `update(dt)` 返回平滑 FPS。

### `control/` — PID 追踪 + 串口通信

- **`config.py`**: `PIDConfig`（kp/ki/kd + 积分限幅/输出限幅）和 `ControlConfig`（死区/丢目标超时/最大 RPM/方向反转 per 轴）。yaw 和 pitch 各有独立 PID 参数。
- **`pid.py`**: 标准 PID 实现，含积分抗饱和（`integral_limit`）和输出限幅（`output_limit`），微分项用 error 差分。输入/输出无量纲，由上层缩放。
- **`tracker_control.py`**: `GimbalTracker.update()` 接收目标中心像素坐标（可为 None），计算归一化误差 `(cx - w/2) / (w/2)`，经死区处理后送双轴 PID，输出缩放为 `yaw_rpm` / `pitch_rpm`。丢目标时在 `lost_timeout_s` 宽限期内保持上次输出，超时后 reset PID。
- **`serial_stub.py`**: 二进制协议串口通信。pyserial 延迟导入，port 为 None 时静默跳过。`GimbalSerial` 提供 `enable/speed_ctrl/angle_ctrl/query` 等高层方法。协议定义见下位机 `Applications/Src/TransmitTask.cpp`。

### 串口协议（与 STM32 QGimbal 通信）

二进制协议：USART6 (PG14/PG9) 115200 8N1，CRC8(poly=0x07, init=0, xor_out=0, 无位反转)。

**上位机 → 下位机（10 字节）**：

| 偏移 | 大小 | 内容 |
|------|------|------|
| 0 | 1B | 命令码 (uint8) |
| 1 | 4B | yaw 参数 (float32 LE) |
| 5 | 4B | pitch 参数 (float32 LE) |
| 9 | 1B | CRC8（覆盖前 9 字节） |

**命令码**（与下位机 `CmdType` 枚举一致）：

| 码 | 助记符 | 功能 |
|----|--------|------|
| 0x00 | NOP | 查询状态（无操作） |
| 0x01 | Enable | 使能电机 |
| 0x02 | Disable | 失能电机 |
| 0x03 | CurrentCtrl | 电流环控制 (A) |
| 0x04 | SpeedCtrl | 速度环控制 (RPM) |
| 0x05 | AngleCtrl | 角度环控制 (rad) |
| 0x06 | LowSpeedCtrl | 低速控制 (RPM) |
| 0x07 | StepAngleCtrl | 步进角度控制 |
| 0xFB | ResetIMU | 复位 IMU 零点 |
| 0xFC | DisableLaser | 关闭激光 |
| 0xFD | EnableLaser | 打开激光 |
| 0xFE | DisableStability | 关闭自稳 |
| 0xFF | EnableStability | 开启自稳 |

**下位机 → 上位机（42 字节）**：

| 偏移 | 大小 | 内容 |
|------|------|------|
| 0 | 1B | status (bit0:en, bit1:stb, bit2:las) |
| 1 | 4B | imu_speed_yaw (RPM, f32 LE) |
| 5 | 4B | imu_speed_pitch (RPM, f32 LE) |
| 9 | 4B | imu_angle_yaw (rad, f32 LE) |
| 13 | 4B | imu_angle_pitch (rad, f32 LE) |
| 17 | 4B | motor_current_yaw (A, f32 LE) |
| 21 | 4B | motor_current_pitch (A, f32 LE) |
| 25 | 4B | motor_speed_yaw (RPM, f32 LE) |
| 29 | 4B | motor_speed_pitch (RPM, f32 LE) |
| 33 | 4B | motor_angle_yaw (rad, f32 LE) |
| 37 | 4B | motor_angle_pitch (rad, f32 LE) |
| 41 | 1B | CRC8（覆盖前 41 字节） |

**设计决策**：
- `speed_ctrl()` 为 fire-and-forget（仅发送，不读回复），适应 30fps 高频调用。每个命令 STM32 都会回复，串口驱动缓冲区自动接收。
- 其他命令（`enable`、`disable`、`query` 等）为同步模式：发送后阻塞读取 42 字节回复。
- **关键前置条件**：STM32 `CommunicateTask` 等待 `qgimbal.enabled` 后才初始化 UART DMA，云台使能依赖 IMU (BMI088) 成功初始化。IMU 不工作 → 云台不使能 → 串口无响应。

### 坐标系约定

- 图像坐标：x 向右为正，y 向下为正
- 误差定义：`err = target_center - image_center`
- 输出方向未知时用 `ControlConfig.invert_yaw` / `invert_pitch` 反转

### 关键设计决策

- 摄像头默认强制 V4L2 后端（`cv2.CAP_V4L2`），适应 Linux USB UVC 摄像头；若失败回退默认后端。
- 矩形检测在 `main.py` 中取列表首项（最大面积）作为 `best`，不需要额外逻辑选择。
- PID 归一化设计：输入误差归一化到 [-1, 1]，输出也归一化到 [-1, 1]，由 `max_rpm_*` 缩放为实际 RPM，使 PID 参数与图像分辨率无关。
- 串口模块无运行时强制依赖，`import serial` 仅在 `port` 非 None 时执行。
- `main.py` 对每帧做 `cv2.flip(frame, -1)`（上下+左右翻转），以适应摄像头实际安装方向。

### 关联仓库

下位机固件位于 `../QGimbal-master/`（STM32F407 + FreeRTOS），其 `CLAUDE.md` 包含完整的启动流程、任务架构、控制管线和配置参数。两个仓库通过 USART6 (PG14/PG9) 115200 8N1 通信。
