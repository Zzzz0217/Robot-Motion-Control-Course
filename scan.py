import serial
import struct
import argparse
import time
import cv2
import numpy as np
from pyzbar import pyzbar

# 协议常量
FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D

# 角度环 PID 参数
Kp_angle = 0.0018
Ki_angle = 0.00005
Kd_angle = 0.0007

# 速度环 PID 参数
Kp_speed = 0.0025  # 速度比例系数
Ki_speed = 0.00005  # 速度积分系数
Kd_speed = 0.00005  # 速度微分系数

# 积分和上一次误差
integral_angle = 0
previous_error_angle = 0
integral_speed = 0
previous_error_speed = 0

MAX_ANGULAR_SPEED = 4.0  # 最大角速度 (rad/s)

def build_frame(vx: float, vy: float, vz: float) -> bytes:
    """构造14字节帧：[HEADER(1)] [vx(4)] [vy(4)] [vz(4)] [TAIL(1)]"""
    frame = bytearray()
    frame.append(FRAME_HEADER)
    frame += struct.pack('<f', vx)
    frame += struct.pack('<f', vy)
    frame += struct.pack('<f', vz)
    frame.append(FRAME_TAIL)
    return bytes(frame)

# ───────── 摄像头 & 推流参数 ─────────
DEVICE = 0
TARGET_W = 1280
TARGET_H = 720
TARGET_FPS = 30
JPEG_QUALITY = 70

# ───────── 车道线算法参数 ─────────
ROI_RATIO = 0.7   # 0.5 = 下半幅
NUM_SLICES = 6
VALID_AREA = 1000      # 最小有效面积
MAX_VALID_AREA = 15000  # 新增：最大有效面积
HSV_LOWER = np.array([0, 0, 0], np.uint8)
HSV_UPPER = np.array([180, 255, 60], np.uint8)
KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

# ───────── 速度环控制参数 ─────────
MAX_SPEED = 0.6  # 最大速度 (m/s)
MIN_SPEED = 0.001  # 最小速度 (m/s)
ANGLE_THRESHOLD = 0.5  # 角度阈值，用于速度调节

# ───────── 初始化摄像头 ─────────
cap = None

def initialize_camera():
    """初始化摄像头并返回摄像头对象"""
    global cap
    cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError("无法打开摄像头")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, TARGET_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TARGET_H)
    cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    src_w, src_h = int(cap.get(3)), int(cap.get(4))
    src_fps = cap.get(cv2.CAP_PROP_FPS) or 0
    print(f"Camera native {src_w}×{src_h}  {src_fps:.1f} FPS")
    return cap

# ───────── 车道线检测 ─────────
def detect_lane(frame):
    h, w = frame.shape[:2]
    roi_y0 = int(h * (1 - ROI_RATIO))
    slice_h = (h - roi_y0) // NUM_SLICES

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, 1)
    mask = cv2.dilate(mask, KERNEL, 1)

    center_x = 0
    valid_count = 0
    for i in range(NUM_SLICES):
        y1 = roi_y0 + i * slice_h
        y2 = h if i == NUM_SLICES - 1 else roi_y0 + (i + 1) * slice_h
        slice_mask = mask[y1:y2]

        cnts, _ = cv2.findContours(slice_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            continue
        largest = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        
        # 同时检查最小和最大有效面积
        if area < VALID_AREA or area > MAX_VALID_AREA:
            continue

        x, y, w_box, h_box = cv2.boundingRect(largest)
        cx, cy = x + w_box // 2, y + h_box // 2 + y1
        center_x += cx
        valid_count += 1

    if valid_count > 0:
        center_x /= valid_count
    return center_x

def pid_control_angle(center_x, image_width):
    """角度环PID控制"""
    global integral_angle, previous_error_angle
    setpoint = image_width / 2
    error = setpoint - center_x

    # 积分限幅
    integral_max = 10000
    integral_min = -10000
    integral_angle += error
    integral_angle = max(min(integral_angle, integral_max), integral_min)

    # 计算微分项
    derivative = error - previous_error_angle

    # 计算PID输出
    output = Kp_angle * error + Ki_angle * integral_angle + Kd_angle * derivative

    # 输出限幅
    output_max = 100.0
    output_min = -100.0
    output = max(min(output, output_max), output_min)

    # 更新上一次误差
    previous_error_angle = error

    print(f"Angle PID: Error={error:.2f}, Integral={integral_angle:.2f}, Derivative={derivative:.2f}, Output={output:.2f}")

    return output

def pid_control_speed(target_speed, current_speed):
    """速度环PID控制"""
    global integral_speed, previous_error_speed
    error = target_speed - current_speed

    # 积分限幅
    integral_max = 400
    integral_min = -400
    integral_speed += error
    integral_speed = max(min(integral_speed, integral_max), integral_min)

    # 计算微分项
    derivative = error - previous_error_speed

    # 计算PID输出
    output = Kp_speed * error + Ki_speed * integral_speed + Kd_speed * derivative

    # 输出限幅
    output_max = 0.5  # 速度调整范围
    output_min = -0.5
    output = max(min(output, output_max), output_min)

    # 更新上一次误差
    previous_error_speed = error

    print(f"Speed PID: Error={error:.2f}, Integral={integral_speed:.2f}, Derivative={derivative:.2f}, Output={output:.2f}")

    return output

def cleanup(ser):
    """清理资源并重置系统状态"""
    global integral_angle, previous_error_angle, integral_speed, previous_error_speed
    
    # 发送停止指令
    print("发送停止指令...")
    frame_to_send = build_frame(0, 0, 0)
    ser.write(frame_to_send)
    
    # 释放摄像头资源
    if cap is not None and cap.isOpened():
        print("释放摄像头资源...")
        cap.release()
    
    # 重置PID控制参数
    print("重置PID控制参数...")
    integral_angle = 0
    previous_error_angle = 0
    integral_speed = 0
    previous_error_speed = 0
    
    print("系统已完全重置")

def detect_qr_code(frame):
    """检测二维码"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    decoded_objects = pyzbar.decode(gray)
    return len(decoded_objects) > 0

def main(port: str, baudrate: int, vx_base: float, rate_hz: float = 50.0, timeout: float = 1.0) -> None:
    period = 1.0 / rate_hz  # 控制周期

    # 初始化摄像头
    camera = initialize_camera()

    # 初始化当前速度（假设初始为0）
    current_speed = 0.0

    # 记录上一次检测到车道线时的状态
    last_valid_vx = 0.0
    last_valid_vz = 0.0
    # 记录连续丢线次数
    lost_line_count = 0
    # 最大连续丢线尝试次数
    MAX_LOST_LINE_TRIES = 10  

    with serial.Serial(port, baudrate, timeout=timeout) as ser:
        print(f'已打开 {port}，波特率 {baudrate}，发送频率 {rate_hz} Hz')
        try:
            while True:
                ok, frame = camera.read()
                if not ok:
                    continue

                # 检测二维码
                if detect_qr_code(frame):
                    print("检测到二维码，停车")
                    frame_to_send = build_frame(0, 0, 0)
                    ser.write(frame_to_send)
                    break

                center_x = detect_lane(frame)

                if center_x != 0:
                    # 检测到车道线，更新状态
                    lost_line_count = 0
                    # 角度环PID控制
                    vz = pid_control_angle(center_x, frame.shape[1])

                    # 计算角度绝对值（用于速度调节）
                    angle_abs = abs(vz)

                    # 根据角度调整目标速度（弯道减速，直道加速）
                    if angle_abs > ANGLE_THRESHOLD:
                        # 弯道减速
                        slow_factor = 1.0 - min(1.0, angle_abs / 2.0)  # 最大减速到0
                        target_speed = vx_base * slow_factor
                        target_speed = max(target_speed, MIN_SPEED)  # 确保不低于最小速度
                    else:
                        # 直道加速（但不超过最大速度）
                        target_speed = min(vx_base, MAX_SPEED)

                    # 速度环PID控制
                    speed_output = pid_control_speed(target_speed, current_speed)

                    # 最终速度 = 基础速度 + PID调整值
                    vx = target_speed + speed_output
                    vx = max(MIN_SPEED, min(MAX_SPEED, vx))  # 确保速度在合理范围内

                    # 更新上一次有效状态
                    last_valid_vx = vx
                    last_valid_vz = vz

                else:
                    lost_line_count += 1
                    if lost_line_count <= MAX_LOST_LINE_TRIES:
                        print("未检测到车道线，以最大角速度回线。")
                        time.sleep(0.1)  # 等待一段时间后再尝试回线
                        # 使用最后一次检测到的车道线位置来确定回线方向
                        if last_valid_vz != 0:
                            # 保持最后角速度方向，但使用最大角速度
                            vz = MAX_ANGULAR_SPEED if last_valid_vz > 0 else -MAX_ANGULAR_SPEED
                        else:
                            # 如果之前没有有效角速度信息，默认向右转
                            vz = MAX_ANGULAR_SPEED
                        # 丢线时降低前进速度
                        vx = MIN_SPEED
                    else:
                        print("多次尝试回线失败，继续沿上一状态行驶。")
                        vx = last_valid_vx
                        vz = last_valid_vz

                print(f"vx_base={vx_base:.2f}, target_speed={target_speed:.2f}, vx={vx:.2f}, vz={vz:.2f}")

                # 发送速度帧
                frame_to_send = build_frame(vx, 0, vz)
                ser.write(frame_to_send)

                # 更新当前速度（实际应用中应通过编码器等传感器获取）
                current_speed = vx  # 简化处理，实际应读取传感器

                time.sleep(period)
        except KeyboardInterrupt:
            print('\n中断，正在清理资源...')
            cleanup(ser)
        except Exception as e:
            print(f"发生错误: {e}")
            cleanup(ser)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='基于PID控制的巡线')
    parser.add_argument('--port', default='/dev/ttyCH343USB0', help='串口号 (如 COM7 或 /dev/ttyUSB0)')
    parser.add_argument('--baudrate', type=int, default=115200, help='波特率')
    parser.add_argument('--vx', type=float, default=0.05, help='基础前进速度 (m/s)')
    args = parser.parse_args()

    main(args.port, args.baudrate, args.vx)