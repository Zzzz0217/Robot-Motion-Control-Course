#!/usr/bin/env python3
"""
MiniCar 巡线与二维码识别系统
-----------------------------------------
功能：
* 优先巡黑色线，丢线时启用容错策略
* 检测到红色车道线时触发减速，切换保守PID参数
* 仅在红色线区域激活二维码识别，减少资源占用
* 支持透视变换增强二维码解码准确率
"""

import os
import time
import cv2
import numpy as np
import concurrent.futures as futures
from pyzbar import pyzbar

# ──────────────── 配置参数 ────────────────
# 摄像头参数
CAM_DEVICE = int(os.getenv("CAM_DEVICE", 0))
CAM_WIDTH = int(os.getenv("CAM_WIDTH", 320))
CAM_HEIGHT = int(os.getenv("CAM_HEIGHT", 240))
CAM_FPS = int(os.getenv("CAM_FPS", 30))

# 巡线参数（黑色线）
BLACK_LOW = np.array([0, 0, 0])
BLACK_HIGH = np.array([180, 255, 30])
LINE_REGION_Y = 180  # 巡线检测区域Y坐标（ROI）

# 红色线检测参数
RED_LOW1 = np.array([0, 100, 100])
RED_HIGH1 = np.array([10, 255, 255])
RED_LOW2 = np.array([160, 100, 100])
RED_HIGH2 = np.array([180, 255, 255])
RED_DETECT_THRESH = 50  # 红色像素阈值

# PID参数（Kp, Ki, Kd）
NORMAL_PID = (0.8, 0.01, 0.2)     # 正常巡线PID
CONSERVATIVE_PID = (0.5, 0.005, 0.1)  # 保守模式PID

# 二维码识别参数
QR_THREADS = 2
WARP_SIZE = 200
QR_ENABLE_DIST = 100  # 红色线距离阈值时启用二维码

# 车辆控制参数
MAX_SPEED = 60
DECEL_SPEED = 30      # 红色线减速后的速度
LOST_LINE_SPEED = 20  # 丢线时速度

# ──────────────── 初始化组件 ────────────────
# 摄像头
cap = cv2.VideoCapture(CAM_DEVICE)
if not cap.isOpened():
    raise RuntimeError("无法打开摄像头")
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
cap.set(cv2.CAP_PROP_FPS, CAM_FPS)

# 线程池
executor = futures.ThreadPoolExecutor(max_workers=QR_THREADS)

# 状态变量
in_red_zone = False         # 是否处于红色线区域
last_line_pos = CAM_WIDTH // 2  # 上次黑线位置
lost_line_count = 0         # 丢线计数
current_pid = NORMAL_PID    # 当前PID参数
current_speed = MAX_SPEED   # 当前车速

# ──────────────── 工具函数 ────────────────
def detect_black_line(frame):
    """检测黑色线位置，返回中线偏移量（正值右偏，负值左偏）"""
    # 提取ROI区域
    roi = frame[LINE_REGION_Y:, :]
    h, w = roi.shape[:2]
    
    # 二值化黑色线
    mask = cv2.inRange(roi, BLACK_LOW, BLACK_HIGH)
    # 计算轮廓矩
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None  # 未检测到黑线
    
    # 取最大轮廓
    largest_contour = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest_contour)
    
    if M["m00"] == 0:
        return None
    
    # 计算中心点
    cx = int(M["m10"] / M["m00"])
    line_pos = cx - (w // 2)  # 相对于中线的偏移量
    return line_pos

def detect_red_line(frame):
    """检测红色线，返回是否存在及红色区域强度"""
    # 转换到HSV色彩空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # 红色在HSV中的两个区间（0-10和160-180）
    mask1 = cv2.inRange(hsv, RED_LOW1, RED_HIGH1)
    mask2 = cv2.inRange(hsv, RED_LOW2, RED_HIGH2)
    mask = cv2.bitwise_or(mask1, mask2)
    
    # 计算红色像素数量
    red_pixels = cv2.countNonZero(mask)
    # 仅检测下半部分区域（避免背景干扰）
    roi = mask[LINE_REGION_Y:, :]
    red_roi_pixels = cv2.countNonZero(roi)
    
    # 判断是否处于红色区域
    is_red = red_roi_pixels > RED_DETECT_THRESH
    return is_red, red_roi_pixels

def perspective_transform(image, points):
    """对二维码区域进行透视变换"""
    pts = np.array([
        (points[0].x, points[0].y),
        (points[1].x, points[1].y),
        (points[2].x, points[2].y),
        (points[3].x, points[3].y)
    ], dtype=np.float32)
    
    dst = np.array([
        [0, 0],
        [WARP_SIZE - 1, 0],
        [WARP_SIZE - 1, WARP_SIZE - 1],
        [0, WARP_SIZE - 1]
    ], dtype=np.float32)
    
    M = cv2.getPerspectiveTransform(pts, dst)
    return cv2.warpPerspective(image, M, (WARP_SIZE, WARP_SIZE))

def decode_qr_code(gray_frame):
    """解码二维码，返回解码内容"""
    decoded = executor.submit(pyzbar.decode, gray_frame).result()
    valid_codes = []
    for obj in decoded:
        if len(obj.polygon) == 4:
            # 透视变换增强解码
            warped = perspective_transform(gray_frame, obj.polygon)
            warped = cv2.equalizeHist(warped)
            warped_decoded = executor.submit(pyzbar.decode, warped).result()
            if warped_decoded and warped_decoded[0].data:
                valid_codes.append(warped_decoded[0].data.decode("utf-8", "ignore"))
    return valid_codes if valid_codes else None

# ──────────────── 车辆控制逻辑 ────────────────
def update_control(line_pos, red_strength):
    """根据线位置和红色强度更新车辆控制参数"""
    global in_red_zone, last_line_pos, lost_line_count
    global current_pid, current_speed
    
    # 1. 处理红色线检测
    if red_strength > QR_ENABLE_DIST:
        in_red_zone = True
        current_speed = DECEL_SPEED
        current_pid = CONSERVATIVE_PID
    else:
        in_red_zone = False
        current_speed = MAX_SPEED
        current_pid = NORMAL_PID
    
    # 2. 处理黑线检测结果
    if line_pos is not None:
        # 检测到黑线，重置丢线计数
        lost_line_count = 0
        last_line_pos = line_pos
    else:
        # 丢线处理
        lost_line_count += 1
        if lost_line_count < 5:
            # 短时间丢线，使用上次位置
            line_pos = last_line_pos
        else:
            # 长时间丢线，减速并启用保守策略
            current_speed = LOST_LINE_SPEED
            line_pos = 0  # 假设中线位置
    
    # 3. PID计算转向（简化示例，实际需对接电机控制）
    kp, ki, kd = current_pid
    steering = kp * line_pos
    print(f"车速: {current_speed}, 转向: {steering:.1f}, "
          f"红线强度: {red_strength}, 丢线计数: {lost_line_count}")
    
    # （此处应添加实际电机控制代码）
    return current_speed, steering

# ──────────────── 主循环 ────────────────
def main_loop():
    """系统主循环：巡线+二维码识别"""
    print("系统启动，开始巡线...")
    last_qr_time = 0
    
    while True:
        t_start = time.time()
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.1)
            continue
        
        # 1. 检测黑色线
        line_pos = detect_black_line(frame)
        
        # 2. 检测红色线
        is_red, red_strength = detect_red_line(frame)
        
        # 3. 更新车辆控制
        speed, steering = update_control(line_pos, red_strength)
        
        # 4. 红色区域启用二维码识别（避免频繁解码）
        if in_red_zone and (time.time() - last_qr_time > 1):
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            qr_data = decode_qr_code(gray)
            if qr_data:
                print(f"[QR识别] 内容: {qr_data[0]}")
                last_qr_time = time.time()
        
        # 5. 性能统计（可选）
        process_time = (time.time() - t_start) * 1000
        # print(f"处理耗时: {process_time:.1f}ms")
        
        # 按ESC键退出
        if cv2.waitKey(1) == 27:
            break

if __name__ == '__main__':
    try:
        main_loop()
    except KeyboardInterrupt:
        print("\n系统停止")
    finally:
        cap.release()
        executor.shutdown(wait=False)