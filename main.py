#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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
# 创建Flask应用实例
app = Flask(__name__)
# 定义根路由，返回HTML页面
@app.route('/')
def index():
    # 定义HTML模板字符串
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
    # 使用模板字符串渲染并返回HTML页面
    return render_template_string(html)
# ───────── ANSI 彩色输出定义 ─────────
# 重置颜色
COLOR_RESET = "\033[0m"
COLOR_RED   = "\033[31m"   
COLOR_BLUE  = "\033[34m"   
COLOR_GREEN = "\033[32m"   
COLOR_YELLOW = "\033[33m"  
COLOR_PURPLE = "\033[35m"  
# ───────── 调试控制 ─────────
# 调试开关
DEBUG = True
# 控制小车运行状态的全局变量
CAR_RUNNING = False  
# 调试配置字典
DEBUG_CONFIG = {
    # 类别: 输出间隔帧数
    # 每隔n帧输出一次调试信息
    "frame": 1,       
    # "lane": 每隔n帧输出一次车道线调试信息
    "lane": 5,      
    # "PID"：每隔n帧输出一次pid调试信息  
    "pid": 10,        
    # "serial": 每隔n帧输出一次串口调试信息
    "serial": 20,     
    # "qr": 每隔n帧输出一次二维码调试信息
    "qr": 1,          
    # "decision": 每隔n帧输出一次决策调试信息
    "decision": 1     
}
# 初始化每个调试类别的帧计数器
_frame_counters = {category: 0 for category in DEBUG_CONFIG}
# 调试信息输出函数
def debug(msg, category="default"):
    global _frame_counters
    if DEBUG:
        # 获取调试信息输出间隔
        interval = DEBUG_CONFIG.get(category, DEBUG_CONFIG.get("default", 1))
        # 对应类别的帧计数器加1
        _frame_counters[category] += 1
        if _frame_counters[category] % interval == 0:
            # 输出调试信息
            print(f"{COLOR_PURPLE}[DEBUG][{category}] {msg}{COLOR_RESET}")
            if _frame_counters[category] >= 1000000:
                # 防止计数器溢出，重置为0
                _frame_counters[category] = 0
# ───────── 摄像头 & 控制参数 ─────────
# 摄像头设备编号
DEVICE        = int(os.getenv("CAM_DEVICE", 0))
# 摄像头分辨率
CAM_WIDTH     = int(os.getenv("CAM_WIDTH", 640))
CAM_HEIGHT    = int(os.getenv("CAM_HEIGHT", 480))
# 摄像头帧率
CAM_FPS       = int(os.getenv("CAM_FPS", 15))
CONTROL_FPS   = int(os.getenv("CONTROL_FPS", 15))
FRAME_INTERVAL = 1.0 / CONTROL_FPS
# ───────── Web 可视化参数 ─────────
JPEG_QUALITY = int(os.getenv("JPEG_QUALITY", 70))
# Web流帧率
STREAM_FPS = int(os.getenv("STREAM_FPS", 15))
FRAME_INTERVAL_WEB = 1.0 / STREAM_FPS
# Web状态信息字典
STATS = {
    "cam_fps": CAM_FPS,
    "proc_fps": 0.0,
    "latency_ms": 0.0,
    "cam_resolution": f"{CAM_WIDTH}×{CAM_HEIGHT}",
    "status": "",
    "qr": "",
}
# Web流帧计数器
_cnt_web = 0
_t0_web = time.time()
_last_web = 0
_cnt_web, _t0_web, _last_web = 0, time.time(), 0
# 重新创建Flask应用实例
app = Flask(__name__)
# 车道线感兴趣区域比例
ROI_RATIO  = float(os.getenv("LANE_ROI_RATIO", 0.5))
# 车道线分割切片数量
NUM_SLICES = int(os.getenv("LANE_SLICES", 5))
# 车道线有效轮廓面积最小值
VALID_AREA = int(os.getenv("LANE_MIN_AREA", 1500))
# 车道线有效轮廓面积最大值
MAX_AREA   = int(os.getenv("LANE_MAX_AREA", 50000))
# 车道线HSV颜色下限
HSV_LOWER = np.array([0, 0, 0],   np.uint8)
# 车道线HSV颜色上限
HSV_UPPER = np.array([180, 255, 80], np.uint8)
# 形态学操作结构元素
KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
# ───────── 红色车道线参数 ─────────
# 红色车道线感兴趣区域比例
RED_ROI_RATIO = 0.7
# 红色车道线分割切片数量
RED_NUM_SLICES = 6
# 红色车道线有效轮廓面积最小值
RED_VALID_AREA = 1000
# 红色车道线有效轮廓面积最大值
RED_MAX_VALID_AREA = 15000
# 红色车道线HSV颜色下限1
RED_HSV_LOWER = np.array([0, 120, 70], np.uint8)
# 红色车道线HSV颜色上限1
RED_HSV_UPPER = np.array([10, 255, 255], np.uint8)
# 红色车道线HSV颜色下限2
RED_HSV_LOWER2 = np.array([160, 120, 70], np.uint8)
# 红色车道线HSV颜色上限2
RED_HSV_UPPER2 = np.array([180, 255, 255], np.uint8)
# 红色车道线形态学操作结构元素
RED_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
# ───────── 串口协议常量 ─────────
FRAME_HEADER = 0x7B
FRAME_TAIL   = 0x7D
# ───────── PID 控制器 （角度单位：度） ─────────
class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, dt: float, name: str = "PID", 
                 max_integral=100.0, min_output=None, max_output=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0
        self.name = name
        self.max_integral = max_integral  
        self.min_output = min_output      
        self.max_output = max_output      
        # 保存最近的误差，用于检测异常和滤波
        self.history = []                 
        # 历史记录长度
        self.history_size = 5             
        # 输出控制器初始化信息
        debug(f"初始化 {self.name} 控制器: Kp={kp}, Ki={ki}, Kd={kd}, dt={dt}", "pid")
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.history = []
        debug(f"{self.name} 控制器已重置", "pid")
    def compute(self, error: float) -> float:
        # 异常值检测：如果误差突变过大，可能是传感器或计算错误
        if len(self.history) > 0:
            # 计算历史误差平均值
            avg_error = sum(self.history) / len(self.history)
            if abs(error - avg_error) > 20.0 and abs(error) > 10.0:  # 异常检测阈值
                debug(f"{self.name} 检测到异常误差: {error:.4f} vs 平均 {avg_error:.4f}, 使用平滑值", "pid")
                error = (error + avg_error) / 2.0  
        self.history.append(error)
        if len(self.history) > self.history_size:
            # 移除最早的误差记录
            self.history.pop(0)
        # 计算比例项
        p_term = self.kp * error
        # 计算积分项，添加积分限幅防止饱和
        self.integral += error * self.dt
        if self.max_integral is not None:
            # 积分限幅
            if self.integral > self.max_integral:
                self.integral = self.max_integral
            elif self.integral < -self.max_integral:
                self.integral = -self.max_integral
        i_term = self.ki * self.integral
        # 计算微分项，添加平滑处理防止噪声放大
        if len(self.history) > 1:
            # 使用最近几个误差计算平均微分，减少噪声影响
            derivative_sum = 0
            for i in range(1, min(len(self.history), 3)):
                derivative_sum += (self.history[-1] - self.history[-1-i]) / (i * self.dt)
            derivative = derivative_sum / min(len(self.history)-1, 2)
        else:
            derivative = (error - self.prev_error) / self.dt
        d_term = self.kd * derivative
        # 更新上一次误差
        self.prev_error = error
        # 计算输出并应用限幅
        output = p_term + i_term + d_term
        if self.min_output is not None and output < self.min_output:
            output = self.min_output
            debug(f"{self.name} 输出限幅(下限): {output:.4f}", "pid")
        if self.max_output is not None and output > self.max_output:
            output = self.max_output
            debug(f"{self.name} 输出限幅(上限): {output:.4f}", "pid")
        debug(f"{self.name} 计算: error={error:.4f}, P={p_term:.4f}, I={i_term:.4f}, D={d_term:.4f}, 输出={output:.4f}", "pid")
        return output
# ───────── 构造并发送 串口 帧 ─────────
def build_frame(vx: float, vy: float, vz: float) -> bytes:
    # 参数安全检查和限制
    try:
        import math
        if not all(isinstance(val, (int, float)) for val in [vx, vy, vz]):
            debug(f"警告: 非数值类型输入, 使用默认值0.0替代", "serial")
            vx, vy, vz = 0.0, 0.0, 0.0
        # 处理NaN或Inf值
        if any(math.isnan(val) or math.isinf(val) for val in [vx, vy, vz]):
            debug(f"警告: 检测到NaN或Inf值, 使用默认值0.0替代", "serial")
            vx = 0.0 if math.isnan(vx) or math.isinf(vx) else vx
            vy = 0.0 if math.isnan(vy) or math.isinf(vy) else vy
            vz = 0.0 if math.isnan(vz) or math.isinf(vz) else vz
        # 构建帧
        frame = bytearray()
        frame.append(FRAME_HEADER)
        frame += struct.pack('<f', float(vx))
        frame += struct.pack('<f', float(vy))
        frame += struct.pack('<f', float(vz))
        frame.append(FRAME_TAIL)
        debug(f"构建帧: vx={vx:.4f}, vy={vy:.4f}, vz={vz:.4f}", "serial")
        return bytes(frame)
    except Exception as e:
        # 异常处理
        debug(f"构建帧异常: {e}, 返回零速度帧", "serial")
        frame = bytearray()
        frame.append(FRAME_HEADER)
        frame += struct.pack('<f', 0.0)
        frame += struct.pack('<f', 0.0)
        frame += struct.pack('<f', 0.0)
        frame.append(FRAME_TAIL)
        return bytes(frame)
# ───────── 全局 资源 ─────────
cap = None
ser = None
pid = None
red_pid = None
ERROR_COUNT = 0  
# 最大重试次数
MAX_RETRIES = 3  
# 重连延迟时间(秒)
RECONNECT_DELAY = 2.0  
# 错误阈值
ERROR_THRESHOLD = 10  
# ————————全局速度——————————
# 线速度
VX_CONST = 0.2   
# 角速度
MAX_VZ   = 2.0   
# 红色车道线线速度
RED_VX_CONST = 0.2
# 角速度
RED_MAX_VZ = 2.0
# ───────── 初始化 摄像头 ─────────
def init_camera(device: int, width: int, height: int, fps: int, retries=MAX_RETRIES):
    global cap
    debug(f"尝试初始化摄像头 {device} (分辨率: {width}×{height}, FPS: {fps})", "frame")
    for attempt in range(retries):
        try:
            if cap is not None:
                try:
                    cap.release()
                except:
                    pass
            cap = cv2.VideoCapture(device)
            if not cap.isOpened():
                raise RuntimeError(f"无法打开摄像头 {device}")
            # 兼容不同OpenCV版本的MJPG设置
            # OpenCV 3.x/4.x: VideoWriter_fourcc 可能在cv2或cv2.VideoWriter中
            try:
                if hasattr(cv2, 'VideoWriter_fourcc'):
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
                elif hasattr(cv2, 'cv') and hasattr(cv2.cv, 'CV_FOURCC'):
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.cv.CV_FOURCC('M','J','P','G'))
            except Exception as e:
                print(f"{COLOR_YELLOW}[WARNING] 设置MJPG格式失败: {e}, 使用默认格式{COLOR_RESET}")
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = cap.get(cv2.CAP_PROP_FPS)
            test_ret, test_frame = cap.read()
            if not test_ret or test_frame is None:
                raise RuntimeError("摄像头无法读取图像")
            debug(f"摄像头初始化成功: 实际分辨率 {actual_width}×{actual_height}, FPS {actual_fps}", "frame")
            return True
        except Exception as e:
            print(f"{COLOR_YELLOW}[WARNING] 摄像头初始化尝试 {attempt+1}/{retries} 失败: {e}{COLOR_RESET}")
            if attempt < retries - 1:
                print(f"{COLOR_YELLOW}[WARNING] {RECONNECT_DELAY}秒后重试...{COLOR_RESET}")
                time.sleep(RECONNECT_DELAY)
            else:
                print(f"{COLOR_RED}[ERROR] 摄像头初始化失败，已达到最大尝试次数{COLOR_RESET}")
                return False
    return False
# ───────── 车道线检测 + 角度误差 计算 ─────────
def compute_angle_error(frame: np.ndarray, roi_ratio, num_slices, valid_area, max_area, hsv_lower, hsv_upper, kernel, color_name: str) -> tuple[float, int]:
    debug(f"开始 {color_name} 车道线检测", "lane")
    h, w = frame.shape[:2]
    roi_y0 = int(h * (1 - roi_ratio))
    slice_h = (h - roi_y0) // num_slices
    # 转换颜色空间到HSV
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
        # 查找轮廓
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
        # 计算质心
        cx = x + w_box // 2
        cy = y + h_box // 2 + y1
        dx = (w / 2.0) - float(cx)
        dy = float(h) - float(cy)
        # 计算角度
        angle_i = np.degrees(np.arctan2(dx, dy))
        sum_abs += abs(angle_i)
        sum_raw += angle_i
        count += 1
        debug(f"切片 {i}: 找到有效轮廓，面积={area}, 质心=({cx}, {cy}), 角度={angle_i:.2f}°", "lane")
    if count == 0:
        debug(f"{color_name} 车道线检测: 未找到有效轮廓", "lane")
        return 0.0, 0
    # 计算平均角度
    avg_abs = sum_abs / count
    avg_raw = sum_raw / count
    # 方向判断
    direction = 1.0 if avg_raw > 1e-6 else (-1.0 if avg_raw < -1e-6 else 0.0)
    debug(f"{color_name} 车道线检测: 找到 {count} 个有效轮廓, 平均角度={avg_raw:.2f}°, 方向={direction}", "lane")
    return avg_abs * direction, count
# ───────── 退出时 发送零速度 并 清理 ─────────
def send_zero_and_cleanup():
    global ser, cap
    for i in range(3):
        try:
            if ser is not None and ser.is_open:
                debug(f"发送零速度帧 (尝试 {i+1}/3)...", "serial")
                zero_frame = build_frame(0.0, 0.0, 0.0)
                ser.write(zero_frame)
                time.sleep(0.1)
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
            time.sleep(0.5)
            if cap is not None:
                cap.release()
        except:
            pass
    # 清理串口资源
    try:
        if ser is not None and ser.is_open:
            debug("关闭串口...", "serial")
            ser.close()
            debug("串口已关闭", "serial")
    except Exception as e:
        print(f"{COLOR_YELLOW}[WARNING] 关闭串口失败: {e}{COLOR_RESET}")
        try:
            time.sleep(0.5)
            if ser is not None and ser.is_open:
                ser.close()
        except:
            pass
    print(f"{COLOR_GREEN}[INFO] 资源清理完成{COLOR_RESET}")
def signal_handler(sig, frame):
    print(f"\n{COLOR_YELLOW}接收到终止信号 {sig}, 正在清理资源...{COLOR_RESET}")
    send_zero_and_cleanup()
    sys.exit(0)
# ───────── 检测二维码 ─────────
def detect_qr_code(frame):
    debug("开始二维码检测...", "qr")
    # 转换为灰度图像
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # 解码二维码
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
    # 车道线颜色
    COLOR_LINE, COLOR_RECT, COLOR_POINT = (0,255,255), (255,0,0), (0,0,255)
    # 线条厚度
    THICK_LINE, POINT_R = 2, 6
    h, w = frame.shape[:2]
    roi_y0 = int(h * (1 - ROI_RATIO))
    slice_h = (h - roi_y0) // NUM_SLICES
    # 转换颜色空间到HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, 1)
    mask = cv2.dilate(mask, KERNEL, 1)
    for i in range(NUM_SLICES):
        y1 = roi_y0 + i * slice_h
        y2 = h if i == NUM_SLICES - 1 else roi_y0 + (i + 1) * slice_h
        slice_mask = mask[y1:y2]
        # 查找轮廓
        cnts, _ = cv2.findContours(slice_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            continue
        largest = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(largest) < VALID_AREA:
            continue
        x, y, w_box, h_box = cv2.boundingRect(largest)
        # 绘制矩形框
        cv2.rectangle(frame, (x, y + y1), (x + w_box, y + y1 + h_box), COLOR_RECT, THICK_LINE)
        cx, cy = x + w_box // 2, y + h_box // 2 + y1
        # 绘制质心点
        cv2.circle(frame, (cx, cy), POINT_R, COLOR_POINT, -1)
    # ==== 分割线 ====
    for i in range(NUM_SLICES + 1):
        y_split = roi_y0 + i * slice_h
        if y_split > h: y_split = h
        # 绘制分割线
        cv2.line(frame, (0, y_split), (w, y_split), COLOR_LINE, THICK_LINE)
    # 绘制ROI分割线
    cv2.line(frame, (0, roi_y0), (w, roi_y0), COLOR_LINE, THICK_LINE)
    return frame
def gen_frames():
    # 初始化帧计数器
    global _cnt_web, _t0_web, _last_web
    # 初始化帧率统计
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
        # 读取一帧
        ok, frame = cap.read() if hasattr(cap, 'read') else (False, None)
        if not ok or frame is None:
            continue
        frame = visualise_lane(frame)
        STATS["latency_ms"] = (time.time() - tic) * 1000
        _cnt_web += 1
        # 计算帧率
        if time.time() - _t0_web >= 1:
            STATS["proc_fps"] = _cnt_web / (time.time() - _t0_web)
            _cnt_web = 0
            _t0_web = time.time()
        ok, buf = cv2.imencode('.jpg', frame, params)
        if not ok:
            continue
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')
# 定义根路由，返回HTML页面
@app.route('/')
def index():
    # 定义HTML模板字符串
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
    # 使用模板字符串渲染并返回HTML页面
    return render_template_string(html)
# 定义视频流路由
@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
# 定义状态信息路由
@app.route('/stats')
def stats():
    return jsonify(STATS)
# 定义控制指令路由
@app.route('/control/<action>')
def control(action):
    global CAR_RUNNING
    if action == 'start':
        # 启动小车
        CAR_RUNNING = True
        return jsonify({"status": "success", "message": "小车已启动"})
    elif action == 'stop':
        # 停止小车
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
# 运行Web服务
def run_web():
    # 启动Flask应用
    app.run('127.0.0.1', 5000, threaded=True)
# ───────── 主循环 ─────────
def main():
    global cap, ser, pid, red_pid, VX_CONST, VALID_AREA, MAX_AREA, DEBUG, ERROR_COUNT, CAR_RUNNING
    parser = argparse.ArgumentParser(
        description='MiniCar 角度 PID 巡线（高级调试版）')
    parser.add_argument('--cam_device', type=int, default=DEVICE)
    parser.add_argument('--cam_width', type=int, default=CAM_WIDTH)
    parser.add_argument('--cam_height', type=int, default=CAM_HEIGHT)
    parser.add_argument('--cam_fps', type=int, default=CAM_FPS)
    # 串口编号
    parser.add_argument('--port', type=str, default='/dev/ttyACM0')
    # 串口波特率
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--vx', type=float, default=VX_CONST)
    # PID 参数
    parser.add_argument('--kp', type=float, default=0.00550)
    parser.add_argument('--ki', type=float, default=0.0000000001)
    parser.add_argument('--kd', type=float, default=0.000250)
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
    # 调试配置
    DEBUG = not args.no_debug
    DEBUG_CONFIG["frame"] = args.debug_frame
    DEBUG_CONFIG["lane"] = args.debug_lane
    DEBUG_CONFIG["pid"] = args.debug_pid
    DEBUG_CONFIG["serial"] = args.debug_serial
    DEBUG_CONFIG["qr"] = args.debug_qr
    DEBUG_CONFIG["decision"] = args.debug_decision
    if DEBUG:
        # 输出调试配置信息
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
        # 初始化摄像头
        init_camera(args.cam_device, args.cam_width, args.cam_height, args.cam_fps)
    except Exception as e:
        # 输出摄像头初始化失败信息
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
    # 红色车道线 PID 参数
    red_pid = PIDController(kp=0.003, ki=0.00001, kd=0.000005, dt=dt, name="红色PID")
    red_pid.reset()
    last_sign = 1.0
    lost_line_count = 0
    detection_status = "初始化"
    # 二维码内容
    qr_content = ""
    global CAR_RUNNING
    # 关闭web时自动发车
    CAR_RUNNING = True  
    try:
        # 上一帧处理时间
        last_time = time.time()
        # 帧计数器
        frame_count = 0
        while True:
            frame_count += 1
            # 帧处理开始时间
            perf_frame_start = time.time()
            now = perf_frame_start
            # 计算时间间隔
            elapsed = now - last_time
            if elapsed < FRAME_INTERVAL:
                time.sleep(FRAME_INTERVAL - elapsed)
            last_time = time.time()
            # 输出帧处理开始信息
            debug(f"\n===== 帧 {frame_count} 处理开始 =====", "frame")
            # 输出时间间隔信息
            debug(f"时间间隔: {elapsed:.4f}s, 目标间隔: {FRAME_INTERVAL:.4f}s", "frame")
            if cap is None:
                # 输出摄像头未初始化信息
                print(f"{COLOR_YELLOW}[WARNING] 摄像头未初始化{COLOR_RESET}")
                time.sleep(0.1)
                continue
            try:
                ok, frame = cap.read() if hasattr(cap, 'read') else (False, None)
                if not ok or frame is None:
                    # 输出读取帧失败信息
                    print(f"{COLOR_YELLOW}[WARNING] 读取帧失败，跳过当前帧{COLOR_RESET}")
                    ERROR_COUNT += 1
                    # 如果连续多次读取失败，尝试重新初始化摄像头
                    if ERROR_COUNT > ERROR_THRESHOLD:
                        # 输出连续多次读取失败信息
                        print(f"{COLOR_YELLOW}[WARNING] 连续多次读取失败，尝试重新初始化摄像头{COLOR_RESET}")
                        init_camera(args.cam_device, args.cam_width, args.cam_height, args.cam_fps)
                        ERROR_COUNT = 0
                    time.sleep(0.1)
                    continue
                else:
                    # 读取成功，重置错误计数
                    ERROR_COUNT = max(0, ERROR_COUNT - 1)
            except Exception as e:
                # 输出读取帧异常信息
                print(f"{COLOR_YELLOW}[WARNING] 读取帧异常: {e}, 跳过当前帧{COLOR_RESET}")
                ERROR_COUNT += 1
                time.sleep(0.1)
                continue
            if not CAR_RUNNING:
                # 更新Web状态信息
                STATS["status"] = "已停止"
                continue
            # 检测二维码
            qr_detected, qr_content = detect_qr_code(frame)
            if qr_detected:
                # 输出检测到二维码信息
                print(f"{COLOR_GREEN}[INFO] 检测到二维码: {qr_content}，停车{COLOR_RESET}")
                frame_to_send = build_frame(0, 0, 0)
                if ser is not None and ser.is_open:
                    ser.write(frame_to_send)
                # 更新Web状态信息
                STATS["status"] = "二维码停车"
                STATS["qr"] = qr_content
                # 停止小车
                CAR_RUNNING = False
                break
            # 计算黑色车道线角度误差
            error, count = compute_angle_error(frame, ROI_RATIO, NUM_SLICES, VALID_AREA, MAX_AREA, HSV_LOWER, HSV_UPPER, KERNEL, "黑色")
            # 计算红色车道线角度误差
            red_error, red_count = compute_angle_error(frame, RED_ROI_RATIO, RED_NUM_SLICES, RED_VALID_AREA, RED_MAX_VALID_AREA, RED_HSV_LOWER, RED_HSV_UPPER, RED_KERNEL, "红色")
            if red_count == 0:
                # 输出红色通道第一次检测失败信息
                debug("红色通道第一次检测失败，尝试第二次检测...", "lane")
                red_error, red_count = compute_angle_error(frame, RED_ROI_RATIO, RED_NUM_SLICES, RED_VALID_AREA, RED_MAX_VALID_AREA, RED_HSV_LOWER2, RED_HSV_UPPER2, RED_KERNEL, "红色(第二次)")
            if red_count > 0:
                # 更新检测状态
                detection_status = "红色车道线"
                # 重置丢失车道线帧数
                lost_line_count = 0
                debug("采用红色车道线保守策略", "decision")
                # 计算红色车道线PID输出
                raw_vz = red_pid.compute(red_error)
                if raw_vz > RED_MAX_VZ:
                    send_vz = RED_MAX_VZ
                    # 输出红色PID输出限幅信息
                    debug(f"红色PID输出限幅: {raw_vz:.4f} → {send_vz:.4f}", "pid")
                elif raw_vz < -RED_MAX_VZ:
                    send_vz = -RED_MAX_VZ
                    # 输出红色PID输出限幅信息
                    debug(f"红色PID输出限幅: {raw_vz:.4f} → {send_vz:.4f}", "pid")
                else:
                    send_vz = raw_vz
                    # 输出红色PID输出信息
                    debug(f"红色PID输出: {send_vz:.4f}", "pid")
                # 设置红色车道线线速度
                send_vx = RED_VX_CONST
                last_sign = 1.0 if red_error > 1e-6 else (-1.0 if red_error < -1e-6 else last_sign)
            elif count > 0:
                # 更新检测状态
                detection_status = "黑色车道线"
                # 重置丢失车道线帧数
                lost_line_count = 0
                debug("采用黑色车道线常规策略", "decision")
                # 计算黑色车道线PID输出
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
                # 设置黑色车道线线速度
                send_vx = VX_CONST
                last_sign = 1.0 if error > 1e-6 else (-1.0 if error < -1e-6 else last_sign)
            else:
                # 丢失车道线帧数加1
                lost_line_count += 1
                # 更新检测状态
                detection_status = f"未检测到车道线（连续{lost_line_count}帧）"
                debug(f"未检测到车道线，采用保持转弯策略，连续丢线次数: {lost_line_count}", "decision")
                # 保持转弯
                send_vx = VX_CONST
                send_vz = last_sign * MAX_VZ
                debug(f"保持转弯: vx={send_vx:.4f}, vz={send_vz:.4f}", "decision")
            print(f"\r{COLOR_GREEN}[INFO]{COLOR_RESET} 状态: {detection_status}   "
                  f"线速度 {COLOR_RED}{send_vx:+.2f} m/s{COLOR_RESET}   "
                  f"角速度 {COLOR_BLUE}{send_vz:+.2f} °/s{COLOR_RESET}", end='')
            vy = 0.0
            # 安全发送控制指令
            serial_write_start = time.time()
            if ser is not None:
                try:
                    frame_to_send = build_frame(send_vx, vy, send_vz)
                    # 检查串口状态
                    if not ser.is_open:
                        # 输出串口未打开信息
                        print(f"{COLOR_YELLOW}[WARNING] 串口未打开，尝试重新打开{COLOR_RESET}")
                        try:
                            ser.open()
                            time.sleep(0.1)
                        except Exception as e:
                            # 输出无法重新打开串口信息
                            print(f"{COLOR_RED}[ERROR] 无法重新打开串口: {e}{COLOR_RESET}")
                    if ser.is_open:
                        # 使用安全发送函数，如果未定义则使用直接发送
                        success = False
                        try:
                            if 'send_serial_frame' in globals():
                                success = send_serial_frame(ser, frame_to_send)
                            else:
                                ser.write(frame_to_send)
                                success = True
                        except Exception as e:
                            # 输出串口发送异常信息
                            print(f"{COLOR_RED}[ERROR] 串口发送异常: {e}{COLOR_RESET}")
                            ERROR_COUNT += 1
                        if success:
                            # 输出控制帧发送成功信息
                            debug("控制帧已成功发送", "serial")
                            ERROR_COUNT = max(0, ERROR_COUNT - 1)  # 成功则减少错误计数
                        elif ERROR_COUNT > ERROR_THRESHOLD:
                            # 连续错误过多，尝试复位通信
                            print(f"{COLOR_YELLOW}[WARNING] 串口通信多次失败，尝试重置连接{COLOR_RESET}")
                            try:
                                ser.close()
                                time.sleep(RECONNECT_DELAY)
                                ser.open()
                                ERROR_COUNT = 0
                            except Exception as e:
                                # 输出重置串口连接失败信息
                                print(f"{COLOR_RED}[ERROR] 重置串口连接失败: {e}{COLOR_RESET}")
                except Exception as e:
                    # 输出串口通信处理异常信息
                    print(f"{COLOR_RED}[ERROR] 串口通信处理异常: {e}{COLOR_RESET}")
            serial_write_end = time.time()
            # 每10帧打印一次性能信息
            if frame_count % 10 == 0:
                print(f"\n[PERF] 帧处理总耗时: {(serial_write_end - perf_frame_start)*1000:.2f} ms, 串口写入耗时: {(serial_write_end - serial_write_start)*1000:.2f} ms")
            # 更新 Web 状态
            STATS["status"] = detection_status
            STATS["qr"] = qr_content if qr_detected else ""
    except Exception as ex:
        # 输出运行异常信息
        print(f"\n{COLOR_RED}[ERROR] 运行异常: {ex}{COLOR_RESET}")
        send_zero_and_cleanup()
        sys.exit(1)
    finally:
        send_zero_and_cleanup()
        print()  # 换行
        # 输出程序安全退出信息
        print(f"{COLOR_GREEN}[INFO] 程序已安全退出{COLOR_RESET}")
if __name__ == '__main__':
    # 是否运行Web服务
    web_run = False
    if web_run:
        # 启动Web服务线程
        threading.Thread(target=run_web, daemon=True).start()
    main()