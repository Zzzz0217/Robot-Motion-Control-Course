#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MiniCar 角度 PID 巡线（高级调试版）+ Web 可视化
--------------------------------------------------------------
• 每帧对图像做车道线检测，支持黑色和红色双线识别；
• 实现独立的 PID 控制器和保守策略；
• 支持细粒度调试信息控制，可通过命令行参数调整不同类型信息的输出频率；
• 支持二维码扫描，扫描成功后停车；
• 支持 Web 实时画面与状态可视化。
"""

import os
import time
import cv2
import numpy as np
import serial
import struct
import argparse
import signal
import sys
from pyzbar import pyzbar
from flask import Flask, Response, render_template_string, jsonify
import threading

app = Flask(__name__)

@app.route('/')
def index():
    html = """<!doctype html><html lang='zh-CN'><head><meta charset='utf-8'>
    <title>MiniCar 巡线+二维码停车 Web可视化</title>
    <style>
      html,body{height:100%;margin:0;overflow:hidden;background:#666;color:#fff;display:flex;flex-direction:column;font-family:sans-serif}
      .title{background:#FFD700;color:#000;text-align:center;font-size:1.4rem;padding:0.6rem 0;}
      .view{flex:1;display:flex;flex-direction:column;align-items:center;gap:20px}
      .camera{flex:1;display:flex;align-items:center;justify-content:center;width:100%}
      .camera img{max-width:100%;max-height:100%;object-fit:contain}
      .controls{display:flex;gap:20px;padding:20px}
      .btn{padding:10px 30px;border:none;border-radius:5px;font-size:1.1rem;cursor:pointer;transition:all 0.3s}
      .btn-start{background:#4CAF50;color:white}
      .btn-start:hover{background:#45a049}
      .btn-stop{background:#f44336;color:white}
      .btn-stop:hover{background:#da190b}
      footer{position:fixed;bottom:0;left:0;width:100%;background:#444;color:#eee;padding:4px 0;font-size:0.9rem;text-align:center}
      footer span{color:#0f0;margin:0 0.3rem}
      #message{position:fixed;top:20px;left:50%;transform:translateX(-50%);
              padding:10px 20px;border-radius:5px;display:none;transition:all 0.3s;
              background:#333;color:white;z-index:1000}
    </style></head><body>
      <div class='title'>MiniCar 巡线+二维码停车 Web可视化</div>
      <div class='view'>
        <div class='camera'><img src='{{ url_for("video_feed") }}' alt='camera'></div>
        <div class='controls'>
          <button class='btn btn-start' onclick='control("start")'>启动巡线</button>
          <button class='btn btn-stop' onclick='control("stop")'>停止小车</button>
        </div>
      </div>
      <div id='message'></div>
      <footer>
        FPS:<span id='cam'>--</span> | 状态:<span id='status'>--</span> | QR:<span id='qr'>--</span> | 延迟:<span id='lat'>--</span>ms | 分辨率:<span id='res'>--</span>
      </footer>
      <script>
        const message = document.getElementById('message');
        function showMessage(msg, isError = false) {
          message.textContent = msg;
          message.style.background = isError ? '#f44336' : '#4CAF50';
          message.style.display = 'block';
          setTimeout(() => { message.style.display = 'none'; }, 3000);
        }
        async function control(action) {
          try {
            const response = await fetch(`/control/${action}`);
            const data = await response.json();
            showMessage(data.message, data.status === 'error');
          } catch(e) {
            showMessage('控制指令发送失败', true);
          }
        }
        async function poll() {
          try {
            const d = await (await fetch('/stats')).json();
            document.getElementById('cam').textContent  = d.cam_fps.toFixed(1);
            document.getElementById('lat').textContent  = d.latency_ms.toFixed(1);
            document.getElementById('res').textContent  = d.cam_resolution;
            document.getElementById('status').textContent = d.status;
            document.getElementById('qr').textContent = d.qr;
          } catch(e) {}
          setTimeout(poll, 1000);
        }
        poll();
      </script></body></html>"""
    return render_template_string(html)

# ───────── ANSI 彩色输出定义 ─────────
COLOR_RESET = "\033[0m"
COLOR_RED   = "\033[31m"   # 用于线速度 vx
COLOR_BLUE  = "\033[34m"   # 用于角速度 vz
COLOR_GREEN = "\033[32m"   # 用于成功信息
COLOR_YELLOW = "\033[33m"  # 用于警告信息
COLOR_PURPLE = "\033[35m"  # 用于调试信息

# ───────── 调试控制 ─────────
DEBUG = True
CAR_RUNNING = False  # 新增：控制小车运行状态的全局变量
DEBUG_CONFIG = {
    "frame": 1,       # 帧处理信息
    "lane": 5,        # 车道线检测信息
    "pid": 10,        # PID控制器信息
    "serial": 20,     # 串口通信信息
    "qr": 1,          # 二维码检测信息
    "decision": 1     # 决策逻辑信息
}

_frame_counters = {category: 0 for category in DEBUG_CONFIG}

def debug(msg, category="default"):
    global _frame_counters
    if DEBUG:
        interval = DEBUG_CONFIG.get(category, DEBUG_CONFIG.get("default", 1))
        _frame_counters[category] += 1
        if _frame_counters[category] % interval == 0:
            print(f"{COLOR_PURPLE}[DEBUG][{category}] {msg}{COLOR_RESET}")
            if _frame_counters[category] >= 1000000:
                _frame_counters[category] = 0

# ───────── 摄像头 & 控制参数 ─────────
DEVICE        = int(os.getenv("CAM_DEVICE", 0))
CAM_WIDTH     = int(os.getenv("CAM_WIDTH", 1280))
CAM_HEIGHT    = int(os.getenv("CAM_HEIGHT", 720))
CAM_FPS       = int(os.getenv("CAM_FPS", 30))
CONTROL_FPS   = int(os.getenv("CONTROL_FPS", 15))
FRAME_INTERVAL = 1.0 / CONTROL_FPS

# ───────── Web 可视化参数 ─────────
JPEG_QUALITY = int(os.getenv("JPEG_QUALITY", 70))
STREAM_FPS = int(os.getenv("STREAM_FPS", 15))
FRAME_INTERVAL_WEB = 1.0 / STREAM_FPS
STATS = {
    "cam_fps": CAM_FPS,
    "proc_fps": 0.0,
    "latency_ms": 0.0,
    "cam_resolution": f"{CAM_WIDTH}×{CAM_HEIGHT}",
    "status": "",
    "qr": "",
}
_cnt_web, _t0_web, _last_web = 0, time.time(), 0
_cnt_web, _t0_web, _last_web = 0, time.time(), 0
app = Flask(__name__)
ROI_RATIO  = float(os.getenv("LANE_ROI_RATIO", 0.35))
NUM_SLICES = int(os.getenv("LANE_SLICES", 5))
VALID_AREA = int(os.getenv("LANE_MIN_AREA", 3000))
MAX_AREA   = int(os.getenv("LANE_MAX_AREA", 20000))

HSV_LOWER = np.array([0, 0, 0],   np.uint8)
HSV_UPPER = np.array([180, 255, 60], np.uint8)
KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

# ───────── 红色车道线参数 ─────────
RED_ROI_RATIO = 0.7
RED_NUM_SLICES = 6
RED_VALID_AREA = 1000
RED_MAX_VALID_AREA = 15000
RED_HSV_LOWER = np.array([0, 120, 70], np.uint8)
RED_HSV_UPPER = np.array([10, 255, 255], np.uint8)
RED_HSV_LOWER2 = np.array([160, 120, 70], np.uint8)
RED_HSV_UPPER2 = np.array([180, 255, 255], np.uint8)
RED_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

# ───────── 串口协议常量 ─────────
FRAME_HEADER = 0x7B
FRAME_TAIL   = 0x7D

# ───────── PID 控制器 （角度单位：度） ─────────
class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, dt: float, name: str = "PID"):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0
        self.name = name
        debug(f"初始化 {self.name} 控制器: Kp={kp}, Ki={ki}, Kd={kd}, dt={dt}", "pid")

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        debug(f"{self.name} 控制器已重置", "pid")

    def compute(self, error: float) -> float:
        p_term = self.kp * error
        self.integral += error * self.dt
        i_term = self.ki * self.integral
        derivative = (error - self.prev_error) / self.dt
        d_term = self.kd * derivative
        self.prev_error = error
        output = p_term + i_term + d_term
        debug(f"{self.name} 计算: error={error:.4f}, P={p_term:.4f}, I={i_term:.4f}, D={d_term:.4f}, 输出={output:.4f}", "pid")
        return output

# ───────── 构造并发送 串口 帧 ─────────
def build_frame(vx: float, vy: float, vz: float) -> bytes:
    frame = bytearray()
    frame.append(FRAME_HEADER)
    frame += struct.pack('<f', vx)
    frame += struct.pack('<f', vy)
    frame += struct.pack('<f', vz)
    frame.append(FRAME_TAIL)
    debug(f"构建帧: vx={vx:.4f}, vy={vy:.4f}, vz={vz:.4f}", "serial")
    return bytes(frame)

# ───────── 全局 资源 ─────────
cap = None
ser = None
pid = None
red_pid = None

# 固定线速度、最大角速度
VX_CONST = 0.5   # m/s
MAX_VZ   = 3.5   # deg/s

# 红色车道线保守参数
RED_VX_CONST = 0.2
RED_MAX_VZ = 2.0

# ───────── 初始化 摄像头 ─────────
def init_camera(device: int, width: int, height: int, fps: int):
    global cap
    debug(f"尝试初始化摄像头 {device} (分辨率: {width}×{height}, FPS: {fps})", "frame")
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        raise RuntimeError(f"无法打开摄像头 {device}")
    # 兼容不同OpenCV版本的MJPG设置
    # OpenCV 3.x/4.x: VideoWriter_fourcc 可能在cv2或cv2.VideoWriter中
    if hasattr(cv2, 'VideoWriter_fourcc'):
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
    elif hasattr(cv2, 'cv') and hasattr(cv2.cv, 'CV_FOURCC'):
        cap.set(cv2.CAP_PROP_FOURCC, cv2.cv.CV_FOURCC('M','J','P','G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    debug(f"摄像头初始化成功: 实际分辨率 {actual_width}×{actual_height}, FPS {actual_fps}", "frame")

# ───────── 车道线检测 + 角度误差 计算 ─────────
def compute_angle_error(frame: np.ndarray, roi_ratio, num_slices, valid_area, max_area, hsv_lower, hsv_upper, kernel, color_name: str) -> tuple[float, int]:
    debug(f"开始 {color_name} 车道线检测", "lane")
    h, w = frame.shape[:2]
    roi_y0 = int(h * (1 - roi_ratio))
    slice_h = (h - roi_y0) // num_slices
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.dilate(mask, kernel)
    sum_abs = 0.0
    sum_raw = 0.0
    count = 0
    for i in range(num_slices):
        y1 = roi_y0 + i * slice_h
        y2 = h if i == num_slices - 1 else (roi_y0 + (i + 1) * slice_h)
        slice_mask = mask[y1:y2]
        cnts, _ = cv2.findContours(slice_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            debug(f"切片 {i}: 未找到轮廓", "lane")
            continue
        largest = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < valid_area or area > max_area:
            debug(f"切片 {i}: 轮廓面积 {area} 不在有效范围内 [{valid_area}, {max_area}]", "lane")
            continue
        x, y, w_box, h_box = cv2.boundingRect(largest)
        cx = x + w_box // 2
        cy = y + h_box // 2 + y1
        dx = (w / 2.0) - float(cx)
        dy = float(h) - float(cy)
        angle_i = np.degrees(np.arctan2(dx, dy))
        sum_abs += abs(angle_i)
        sum_raw += angle_i
        count += 1
        debug(f"切片 {i}: 找到有效轮廓，面积={area}, 质心=({cx}, {cy}), 角度={angle_i:.2f}°", "lane")
    if count == 0:
        debug(f"{color_name} 车道线检测: 未找到有效轮廓", "lane")
        return 0.0, 0
    avg_abs = sum_abs / count
    avg_raw = sum_raw / count
    direction = 1.0 if avg_raw > 1e-6 else (-1.0 if avg_raw < -1e-6 else 0.0)
    debug(f"{color_name} 车道线检测: 找到 {count} 个有效轮廓, 平均角度={avg_raw:.2f}°, 方向={direction}", "lane")
    return avg_abs * direction, count

# ───────── 退出时 发送零速度 并 清理 ─────────
def send_zero_and_cleanup():
    global ser, cap
    try:
        if ser is not None and ser.is_open:
            debug("发送零速度帧...", "serial")
            zero_frame = build_frame(0.0, 0.0, 0.0)
            ser.write(zero_frame)
            time.sleep(0.05)
            debug("零速度帧发送成功", "serial")
    except Exception as e:
        print(f"{COLOR_YELLOW}[WARNING] 发送零速度帧失败: {e}{COLOR_RESET}")
    try:
        if cap is not None:
            debug("释放摄像头资源...", "frame")
            cap.release()
            debug("摄像头资源已释放", "frame")
    except Exception as e:
        print(f"{COLOR_YELLOW}[WARNING] 释放摄像头失败: {e}{COLOR_RESET}")
    try:
        if ser is not None and ser.is_open:
            debug("关闭串口...", "serial")
            ser.close()
            debug("串口已关闭", "serial")
    except Exception as e:
        print(f"{COLOR_YELLOW}[WARNING] 关闭串口失败: {e}{COLOR_RESET}")

def signal_handler(sig, frame):
    print(f"\n{COLOR_YELLOW}接收到终止信号 {sig}, 正在清理资源...{COLOR_RESET}")
    send_zero_and_cleanup()
    sys.exit(0)

# ───────── 检测二维码 ─────────
def detect_qr_code(frame):
    debug("开始二维码检测...", "qr")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    decoded_objects = pyzbar.decode(gray)
    if len(decoded_objects) > 0:
        for obj in decoded_objects:
            content = obj.data.decode("utf-8")
            debug(f"检测到二维码: {content}", "qr")
            return True, content
    debug("未检测到二维码", "qr")
    return False, None

# ───────── Web 可视化 MJPEG流 ─────────
def visualise_lane(frame):
    # 可选：在frame上画出车道线、二维码等
    return frame

def gen_frames():
    global _cnt_web, _t0_web, _last_web
    params = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
    while True:
        dt = time.time() - _last_web
        if dt < FRAME_INTERVAL_WEB:
            time.sleep(FRAME_INTERVAL_WEB - dt)
        _last_web = time.time()
        tic = time.time()
        if cap is None:
            time.sleep(0.1)
            continue
        ok, frame = cap.read() if hasattr(cap, 'read') else (False, None)
        if not ok or frame is None:
            continue
        frame = visualise_lane(frame)
        STATS["latency_ms"] = (time.time() - tic) * 1000
        _cnt_web += 1
        if time.time() - _t0_web >= 1:
            STATS["proc_fps"] = _cnt_web / (time.time() - _t0_web)
            _cnt_web = 0
            _t0_web = time.time()
        ok, buf = cv2.imencode('.jpg', frame, params)
        if not ok:
            continue
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')

@app.route('/')
def index():
    html = """<!doctype html><html lang='zh-CN'><head><meta charset='utf-8'>
    <title>MiniCar 巡线+二维码停车 Web可视化</title>
    <style>
      html,body{height:100%;margin:0;overflow:hidden;background:#666;color:#fff;display:flex;flex-direction:column;font-family:sans-serif}
      .title{background:#FFD700;color:#000;text-align:center;font-size:1.4rem;padding:0.6rem 0;}
      .view{flex:1;display:flex;flex-direction:column;align-items:center;gap:20px}
      .camera{flex:1;display:flex;align-items:center;justify-content:center;width:100%}
      .camera img{max-width:100%;max-height:100%;object-fit:contain}
      .controls{display:flex;gap:20px;padding:20px}
      .btn{padding:10px 30px;border:none;border-radius:5px;font-size:1.1rem;cursor:pointer;transition:all 0.3s}
      .btn-start{background:#4CAF50;color:white}
      .btn-start:hover{background:#45a049}
      .btn-stop{background:#f44336;color:white}
      .btn-stop:hover{background:#da190b}
      footer{position:fixed;bottom:0;left:0;width:100%;background:#444;color:#eee;padding:4px 0;font-size:0.9rem;text-align:center}
      footer span{color:#0f0;margin:0 0.3rem}
      #message{position:fixed;top:20px;left:50%;transform:translateX(-50%);
              padding:10px 20px;border-radius:5px;display:none;transition:all 0.3s;
              background:#333;color:white;z-index:1000}
    </style></head><body>
      <div class='title'>MiniCar 巡线+二维码停车 Web可视化</div>
      <div class='view'>
        <div class='camera'><img src='{{ url_for("video_feed") }}' alt='camera'></div>
        <div class='controls'>
          <button class='btn btn-start' onclick='control("start")'>启动巡线</button>
          <button class='btn btn-stop' onclick='control("stop")'>停止小车</button>
        </div>
      </div>
      <div id='message'></div>
      <footer>
        FPS:<span id='cam'>--</span> | 状态:<span id='status'>--</span> | QR:<span id='qr'>--</span> | 延迟:<span id='lat'>--</span>ms | 分辨率:<span id='res'>--</span>
      </footer>
      <script>
        const message = document.getElementById('message');
        function showMessage(msg, isError = false) {
          message.textContent = msg;
          message.style.background = isError ? '#f44336' : '#4CAF50';
          message.style.display = 'block';
          setTimeout(() => { message.style.display = 'none'; }, 3000);
        }
        async function control(action) {
          try {
            const response = await fetch(`/control/${action}`);
            const data = await response.json();
            showMessage(data.message, data.status === 'error');
          } catch(e) {
            showMessage('控制指令发送失败', true);
          }
        }
        async function poll() {
          try {
            const d = await (await fetch('/stats')).json();
            document.getElementById('cam').textContent  = d.cam_fps.toFixed(1);
            document.getElementById('lat').textContent  = d.latency_ms.toFixed(1);
            document.getElementById('res').textContent  = d.cam_resolution;
            document.getElementById('status').textContent = d.status;
            document.getElementById('qr').textContent = d.qr;
          } catch(e) {}
          setTimeout(poll, 1000);
        }
        poll();
      </script></body></html>"""
    return render_template_string(html)

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/stats')
def stats():
    return jsonify(STATS)

@app.route('/control/<action>')
def control(action):
    global CAR_RUNNING
    if action == 'start':
        CAR_RUNNING = True
        return jsonify({"status": "success", "message": "小车已启动"})
    elif action == 'stop':
        CAR_RUNNING = False
        # 发送停车指令
        if ser is not None and ser.is_open:
            try:
                frame_to_send = build_frame(0, 0, 0)
                ser.write(frame_to_send)
            except Exception as e:
                return jsonify({"status": "error", "message": f"发送停车指令失败: {e}"})
        return jsonify({"status": "success", "message": "小车已停止"})
    return jsonify({"status": "error", "message": "未知命令"})

def run_web():
    app.run('127.0.0.1', 5000, threaded=True)

# ───────── 主循环 ─────────
def main():
    global cap, ser, pid, red_pid, VX_CONST, VALID_AREA, MAX_AREA, DEBUG
    parser = argparse.ArgumentParser(
        description='MiniCar 角度 PID 巡线（高级调试版）')
    parser.add_argument('--cam_device', type=int, default=DEVICE)
    parser.add_argument('--cam_width', type=int, default=CAM_WIDTH)
    parser.add_argument('--cam_height', type=int, default=CAM_HEIGHT)
    parser.add_argument('--cam_fps', type=int, default=CAM_FPS)
    parser.add_argument('--port', type=str, default='/dev/ttyCH343USB0')
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--vx', type=float, default=VX_CONST)
    parser.add_argument('--kp', type=float, default=0.005)
    parser.add_argument('--ki', type=float, default=0.000015)
    parser.add_argument('--kd', type=float, default=0.00001)
    parser.add_argument('--valid_area', type=int, default=VALID_AREA)
    parser.add_argument('--max_area', type=int, default=MAX_AREA)
    parser.add_argument('--no_debug', action='store_true', help='禁用所有调试输出')
    parser.add_argument('--debug_frame', type=int, default=DEBUG_CONFIG["frame"], help='帧处理调试信息输出间隔(帧)')
    parser.add_argument('--debug_lane', type=int, default=DEBUG_CONFIG["lane"], help='车道线调试信息输出间隔(帧)')
    parser.add_argument('--debug_pid', type=int, default=DEBUG_CONFIG["pid"], help='PID调试信息输出间隔(帧)')
    parser.add_argument('--debug_serial', type=int, default=DEBUG_CONFIG["serial"], help='串口调试信息输出间隔(帧)')
    parser.add_argument('--debug_qr', type=int, default=DEBUG_CONFIG["qr"], help='二维码调试信息输出间隔(帧)')
    parser.add_argument('--debug_decision', type=int, default=DEBUG_CONFIG["decision"], help='决策逻辑调试信息输出间隔(帧)')
    args = parser.parse_args()
    DEBUG = not args.no_debug
    DEBUG_CONFIG["frame"] = args.debug_frame
    DEBUG_CONFIG["lane"] = args.debug_lane
    DEBUG_CONFIG["pid"] = args.debug_pid
    DEBUG_CONFIG["serial"] = args.debug_serial
    DEBUG_CONFIG["qr"] = args.debug_qr
    DEBUG_CONFIG["decision"] = args.debug_decision
    if DEBUG:
        print(f"{COLOR_PURPLE}[DEBUG] 调试配置:{COLOR_RESET}")
        for category, interval in DEBUG_CONFIG.items():
            print(f"{COLOR_PURPLE}[DEBUG]   {category}: 每 {interval} 帧输出一次{COLOR_RESET}")
    VALID_AREA = args.valid_area
    MAX_AREA   = args.max_area
    VX_CONST   = args.vx
    print(f"{COLOR_GREEN}[INFO] 启动 MiniCar 巡线系统（高级调试版）{COLOR_RESET}")
    print(f"{COLOR_GREEN}[INFO] PID 参数: KP={args.kp}, KI={args.ki}, KD={args.kd}, 线速度 vx={VX_CONST} m/s{COLOR_RESET}")
    print(f"{COLOR_GREEN}[INFO] 连通域面积范围: [{VALID_AREA}, {MAX_AREA}] 像素{COLOR_RESET}")
    print(f"{COLOR_GREEN}[INFO] 红色车道线保守参数: vx={RED_VX_CONST} m/s, max_vz={RED_MAX_VZ} deg/s{COLOR_RESET}")
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    try:
        init_camera(args.cam_device, args.cam_width, args.cam_height, args.cam_fps)
    except Exception as e:
        print(f"{COLOR_RED}[ERROR] 摄像头初始化失败: {e}{COLOR_RESET}")
        sys.exit(1)
    try:
        ser = serial.Serial(args.port, args.baudrate, timeout=1.0)
        print(f"{COLOR_GREEN}[INFO] 已打开串口 {args.port}, 波特率 {args.baudrate}{COLOR_RESET}")
    except Exception as e:
        ser = None
        print(f"{COLOR_YELLOW}[WARNING] 无法打开串口 {args.port}: {e}, 串口发送功能失效{COLOR_RESET}")
    dt = 1.0 / CONTROL_FPS
    pid = PIDController(kp=args.kp, ki=args.ki, kd=args.kd, dt=dt, name="黑色PID")
    pid.reset()
    red_pid = PIDController(kp=0.003, ki=0.00001, kd=0.000005, dt=dt, name="红色PID")
    red_pid.reset()
    last_sign = 1.0
    lost_line_count = 0
    detection_status = "初始化"
    qr_content = ""
    try:
        last_time = time.time()
        frame_count = 0
        while True:
            frame_count += 1
            now = time.time()
            elapsed = now - last_time
            if elapsed < FRAME_INTERVAL:
                time.sleep(FRAME_INTERVAL - elapsed)
            last_time = time.time()
            debug(f"\n===== 帧 {frame_count} 处理开始 =====", "frame")
            debug(f"时间间隔: {elapsed:.4f}s, 目标间隔: {FRAME_INTERVAL:.4f}s", "frame")
            if cap is None:
                print(f"{COLOR_YELLOW}[WARNING] 摄像头未初始化{COLOR_RESET}")
                time.sleep(0.1)
                continue
            ok, frame = cap.read() if hasattr(cap, 'read') else (False, None)
            if not ok or frame is None:
                print(f"{COLOR_YELLOW}[WARNING] 读取帧失败，跳过当前帧{COLOR_RESET}")
                continue
                
            if not CAR_RUNNING:
                STATS["status"] = "已停止"
                continue
                
            qr_detected, qr_content = detect_qr_code(frame)
            if qr_detected:
                print(f"{COLOR_GREEN}[INFO] 检测到二维码: {qr_content}，停车{COLOR_RESET}")
                frame_to_send = build_frame(0, 0, 0)
                if ser is not None and ser.is_open:
                    ser.write(frame_to_send)
                STATS["status"] = "二维码停车"
                STATS["qr"] = qr_content
                CAR_RUNNING = False
                continue
            error, count = compute_angle_error(frame, ROI_RATIO, NUM_SLICES, VALID_AREA, MAX_AREA, HSV_LOWER, HSV_UPPER, KERNEL, "黑色")
            red_error, red_count = compute_angle_error(frame, RED_ROI_RATIO, RED_NUM_SLICES, RED_VALID_AREA, RED_MAX_VALID_AREA, RED_HSV_LOWER, RED_HSV_UPPER, RED_KERNEL, "红色")
            if red_count == 0:
                debug("红色通道第一次检测失败，尝试第二次检测...", "lane")
                red_error, red_count = compute_angle_error(frame, RED_ROI_RATIO, RED_NUM_SLICES, RED_VALID_AREA, RED_MAX_VALID_AREA, RED_HSV_LOWER2, RED_HSV_UPPER2, RED_KERNEL, "红色(第二次)")
            if red_count > 0:
                detection_status = "红色车道线"
                lost_line_count = 0
                debug("采用红色车道线保守策略", "decision")
                raw_vz = red_pid.compute(red_error)
                if raw_vz > RED_MAX_VZ:
                    send_vz = RED_MAX_VZ
                    debug(f"红色PID输出限幅: {raw_vz:.4f} → {send_vz:.4f}", "pid")
                elif raw_vz < -RED_MAX_VZ:
                    send_vz = -RED_MAX_VZ
                    debug(f"红色PID输出限幅: {raw_vz:.4f} → {send_vz:.4f}", "pid")
                else:
                    send_vz = raw_vz
                    debug(f"红色PID输出: {send_vz:.4f}", "pid")
                send_vx = RED_VX_CONST
                last_sign = 1.0 if red_error > 1e-6 else (-1.0 if red_error < -1e-6 else last_sign)
            elif count > 0:
                detection_status = "黑色车道线"
                lost_line_count = 0
                debug("采用黑色车道线常规策略", "decision")
                raw_vz = pid.compute(error)
                if raw_vz > MAX_VZ:
                    send_vz = MAX_VZ
                    debug(f"黑色PID输出限幅: {raw_vz:.4f} → {send_vz:.4f}", "pid")
                elif raw_vz < -MAX_VZ:
                    send_vz = -MAX_VZ
                    debug(f"黑色PID输出限幅: {raw_vz:.4f} → {send_vz:.4f}", "pid")
                else:
                    send_vz = raw_vz
                    debug(f"黑色PID输出: {send_vz:.4f}", "pid")
                send_vx = VX_CONST
                last_sign = 1.0 if error > 1e-6 else (-1.0 if error < -1e-6 else last_sign)
            else:
                lost_line_count += 1
                detection_status = f"未检测到车道线（连续{lost_line_count}帧）"
                debug(f"未检测到车道线，采用保持转弯策略，连续丢线次数: {lost_line_count}", "decision")
                send_vx = VX_CONST
                send_vz = last_sign * MAX_VZ
                debug(f"保持转弯: vx={send_vx:.4f}, vz={send_vz:.4f}", "decision")
            print(f"\r{COLOR_GREEN}[INFO]{COLOR_RESET} 状态: {detection_status}   "
                  f"线速度 {COLOR_RED}{send_vx:+.2f} m/s{COLOR_RESET}   "
                  f"角速度 {COLOR_BLUE}{send_vz:+.2f} °/s{COLOR_RESET}", end='')
            vy = 0.0
            if ser is not None and ser.is_open:
                try:
                    frame_to_send = build_frame(send_vx, vy, send_vz)
                    ser.write(frame_to_send)
                    debug("控制帧已发送", "serial")
                except Exception as e:
                    print(f"{COLOR_RED}[ERROR] 串口发送异常: {e}{COLOR_RESET}")
            # 更新 Web 状态
            STATS["status"] = detection_status
            STATS["qr"] = qr_content if qr_detected else ""
    except Exception as ex:
        print(f"\n{COLOR_RED}[ERROR] 运行异常: {ex}{COLOR_RESET}")
        send_zero_and_cleanup()
        sys.exit(1)
    finally:
        send_zero_and_cleanup()
        print()  # 换行
        print(f"{COLOR_GREEN}[INFO] 程序已安全退出{COLOR_RESET}")

if __name__ == '__main__':
    threading.Thread(target=run_web, daemon=True).start()
    main()