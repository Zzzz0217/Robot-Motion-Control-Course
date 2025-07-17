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
    def __init__(self, kp: float, ki: float, kd: float, dt: float, name: str = "PID", 
                 max_integral=100.0, min_output=None, max_output=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0
        self.name = name
        self.max_integral = max_integral  # 积分限幅，防止积分饱和
        self.min_output = min_output      # 输出限幅下限
        self.max_output = max_output      # 输出限幅上限
        self.history = []                 # 保存最近的误差，用于检测异常和滤波
        self.history_size = 5             # 历史记录长度
        debug(f"初始化 {self.name} 控制器: Kp={kp}, Ki={ki}, Kd={kd}, dt={dt}", "pid")

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.history = []
        debug(f"{self.name} 控制器已重置", "pid")

    def compute(self, error: float) -> float:
        # 输入验证：确保error是有限的数值
        if not isinstance(error, (int, float)) or np.isnan(error) or np.isinf(error):
            debug(f"{self.name} 收到无效误差值: {error}, 使用0.0替代", "pid")
            error = 0.0

        # 异常值检测和平滑处理
        if len(self.history) > 0:
            avg_error = sum(self.history) / len(self.history)
            # 使用动态阈值，根据历史误差波动范围自适应
            history_std = np.std(self.history) if len(self.history) > 1 else 10.0
            threshold = max(10.0, 3 * history_std)  # 3倍标准差或最小阈值

            if abs(error - avg_error) > threshold:
                debug(f"{self.name} 检测到异常误差: {error:.4f} vs 平均 {avg_error:.4f} (阈值:{threshold:.2f}), 使用平滑值", "pid")
                # 渐进平滑，保留部分新信息但主要使用历史平均
                error = 0.2 * error + 0.8 * avg_error

        # 更新历史记录
        self.history.append(error)
        if len(self.history) > self.history_size:
            self.history.pop(0)

        # 计算比例项
        p_term = self.kp * error

        # 计算积分项，添加积分限幅防止饱和
        self.integral += error * self.dt

        # 积分分离：大偏差时减小积分作用，避免过冲
        if abs(error) > 15.0:  # 积分分离阈值
            integral_factor = 0.5  # 大偏差时降低积分效果
            debug(f"{self.name} 积分分离激活: 因子={integral_factor}", "pid")
        else:
            integral_factor = 1.0

        # 积分限幅
        if self.max_integral is not None:
            if self.integral > self.max_integral:
                self.integral = self.max_integral
                debug(f"{self.name} 积分限幅(上限): {self.integral:.4f}", "pid")
            elif self.integral < -self.max_integral:
                self.integral = -self.max_integral
                debug(f"{self.name} 积分限幅(下限): {self.integral:.4f}", "pid")

        i_term = self.ki * self.integral * integral_factor

        # 计算微分项，添加平滑处理防止噪声放大
        if len(self.history) > 2:
            # 使用中值滤波和加权平均减少异常值影响
            derivatives = []
            weights = []
            for i in range(1, min(len(self.history), 4)):
                der_i = (self.history[-1] - self.history[-1-i]) / (i * self.dt)
                derivatives.append(der_i)
                weights.append(1.0/i)  # 越近的微分权重越高

            # 中值滤波去除异常值
            if len(derivatives) >= 3:
                derivatives.sort()
                derivative = derivatives[len(derivatives)//2]  # 取中值
            else:
                # 加权平均
                derivative = sum(d*w for d,w in zip(derivatives, weights)) / sum(weights)
        else:
            derivative = (error - self.prev_error) / self.dt

        # 微分项平滑和限幅，避免突变
        max_derivative = 50.0  # 最大允许微分值
        if abs(derivative) > max_derivative:
            derivative = max_derivative if derivative > 0 else -max_derivative
            debug(f"{self.name} 微分限幅: {derivative:.4f}", "pid")

        d_term = self.kd * derivative
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
    """增强版帧构建函数，添加了更严格的输入验证和安全限制"""
    try:
        import math

        # 1. 输入类型检查
        if not all(isinstance(val, (int, float)) for val in [vx, vy, vz]):
            debug(f"警告: 非数值类型输入: vx={type(vx)}, vy={type(vy)}, vz={type(vz)}, 使用默认值0.0替代", "serial")
            vx, vy, vz = 0.0, 0.0, 0.0

        # 2. 处理NaN或Inf值
        if any(math.isnan(val) or math.isinf(val) for val in [vx, vy, vz]):
            debug(f"警告: 检测到NaN或Inf值: vx={vx}, vy={vy}, vz={vz}, 使用安全值替代", "serial")
            vx = 0.0 if math.isnan(vx) or math.isinf(vx) else vx
            vy = 0.0 if math.isnan(vy) or math.isinf(vy) else vy
            vz = 0.0 if math.isnan(vz) or math.isinf(vz) else vz

        # 3. 安全限速 - 防止意外的极大值导致系统不稳定
        # 假设系统合理的速度限制
        max_vx = 2.0   # 最大线速度 (m/s)
        max_vy = 2.0   # 最大线速度 (m/s)
        max_vz = 90.0  # 最大角速度 (deg/s)

        if abs(vx) > max_vx:
            debug(f"警告: vx={vx:.4f} 超过安全限制 {max_vx}, 进行限幅", "serial")
            vx = max_vx if vx > 0 else -max_vx

        if abs(vy) > max_vy:
            debug(f"警告: vy={vy:.4f} 超过安全限制 {max_vy}, 进行限幅", "serial")
            vy = max_vy if vy > 0 else -max_vy

        if abs(vz) > max_vz:
            debug(f"警告: vz={vz:.4f} 超过安全限制 {max_vz}, 进行限幅", "serial")
            vz = max_vz if vz > 0 else -max_vz

        # 4. 量化处理 - 减少噪声和抖动
        # 如果值非常接近零，直接设为零
        zero_threshold = 1e-4
        vx = 0.0 if abs(vx) < zero_threshold else vx
        vy = 0.0 if abs(vy) < zero_threshold else vy
        vz = 0.0 if abs(vz) < zero_threshold else vz

        # 5. 构建帧 - 添加异常处理
        try:
            frame = bytearray()
            frame.append(FRAME_HEADER)

            # 对每个pack操作单独进行异常处理
            try:
                frame += struct.pack('<f', float(vx))
            except Exception as e:
                debug(f"警告: vx打包失败: {e}, 使用0.0", "serial")
                frame += struct.pack('<f', 0.0)

            try:
                frame += struct.pack('<f', float(vy))
            except Exception as e:
                debug(f"警告: vy打包失败: {e}, 使用0.0", "serial")
                frame += struct.pack('<f', 0.0)

            try:
                frame += struct.pack('<f', float(vz))
            except Exception as e:
                debug(f"警告: vz打包失败: {e}, 使用0.0", "serial")
                frame += struct.pack('<f', 0.0)

            frame.append(FRAME_TAIL)

            # 验证帧结构
            if len(frame) != 14:  # 头(1) + 3个float(12) + 尾(1)
                raise ValueError(f"帧长度错误: {len(frame)}")

            debug(f"构建帧成功: vx={vx:.4f}, vy={vy:.4f}, vz={vz:.4f}", "serial")
            return bytes(frame)

        except Exception as pack_error:
            debug(f"帧打包异常: {pack_error}, 使用备用方法", "serial")
            # 使用备用方法重新尝试
            frame = bytearray([FRAME_HEADER])
            frame.extend([0] * 12)  # 预留12字节的空间

            # 手动写入浮点数
            try:
                # 不使用struct.pack，手动创建
                frame[1:5] = bytes([0, 0, 0, 0]) if vx == 0 else struct.pack('<f', 0.0)
                frame[5:9] = bytes([0, 0, 0, 0]) if vy == 0 else struct.pack('<f', 0.0)
                frame[9:13] = bytes([0, 0, 0, 0]) if vz == 0 else struct.pack('<f', 0.0)
            except:
                # 如果还是失败，使用全零
                frame[1:13] = bytes([0] * 12)

            frame.append(FRAME_TAIL)
            debug(f"使用备用方法构建零速度帧", "serial")
            return bytes(frame)

    except Exception as e:
        debug(f"构建帧过程中发生未处理异常: {e}, 返回零速度帧", "serial")
        # 最后的安全保障：返回零速度帧
        try:
            frame = bytearray([FRAME_HEADER, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, FRAME_TAIL])
            return bytes(frame)
        except:
            # 如果连简单的bytearray都失败，返回预定义的安全帧
            return bytes([FRAME_HEADER, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, FRAME_TAIL])

# ───────── 全局 资源 ─────────
cap = None
ser = None
pid = None
red_pid = None
ERROR_COUNT = 0  # 错误计数
MAX_RETRIES = 3  # 最大重试次数
RECONNECT_DELAY = 2.0  # 重连延迟时间(秒)
ERROR_THRESHOLD = 10  # 错误阈值

# 固定线速度、最大角速度
VX_CONST = 0.5   # m/s
MAX_VZ   = 3.5   # deg/s

# 红色车道线保守参数
RED_VX_CONST = 0.2
RED_MAX_VZ = 2.0

# ───────── 初始化 摄像头 ─────────
def init_camera(device: int, width: int, height: int, fps: int, retries=MAX_RETRIES):
    """增强版摄像头初始化函数，添加了更全面的错误处理和恢复机制"""
    global cap
    debug(f"尝试初始化摄像头 {device} (分辨率: {width}×{height}, FPS: {fps})", "frame")

    # 参数有效性检查
    if not isinstance(device, int) or device < 0:
        print(f"{COLOR_RED}[ERROR] 无效的摄像头设备号: {device}{COLOR_RESET}")
        return False

    if not (isinstance(width, int) and width > 0 and isinstance(height, int) and height > 0):
        print(f"{COLOR_RED}[ERROR] 无效的分辨率: {width}×{height}{COLOR_RESET}")
        width, height = 640, 480  # 使用安全的默认值
        print(f"{COLOR_YELLOW}[WARNING] 使用安全默认分辨率: {width}×{height}{COLOR_RESET}")

    if not (isinstance(fps, int) and fps > 0):
        print(f"{COLOR_RED}[ERROR] 无效的帧率: {fps}{COLOR_RESET}")
        fps = 30  # 使用安全的默认值
        print(f"{COLOR_YELLOW}[WARNING] 使用安全默认帧率: {fps}{COLOR_RESET}")

    # 退避策略：如果多次尝试失败，增加等待时间
    backoff_time = RECONNECT_DELAY  # 初始等待时间

    for attempt in range(retries):
        try:
            # 安全释放现有资源
            if cap is not None:
                try:
                    for _ in range(3):  # 尝试多次释放
                        try:
                            cap.release()
                            time.sleep(0.2)  # 短暂等待确保资源释放
                            break
                        except Exception as release_error:
                            debug(f"摄像头释放尝试失败: {release_error}", "frame")
                except Exception as final_release_error:
                    print(f"{COLOR_YELLOW}[WARNING] 摄像头资源释放失败: {final_release_error}{COLOR_RESET}")
                cap = None  # 确保引用被清除

            # 在Windows系统上可能需要特殊处理
            if os.name == 'nt':
                # 在Windows上尝试使用特定API
                try:
                    cap = cv2.VideoCapture(device, cv2.CAP_DSHOW)  # 使用DirectShow后端
                except Exception:
                    cap = cv2.VideoCapture(device)  # 回退到默认
            else:
                # 在Linux/其他系统使用默认方式
                cap = cv2.VideoCapture(device)

            if not cap.isOpened():
                raise RuntimeError(f"无法打开摄像头 {device}")

            # 兼容不同OpenCV版本的MJPG设置
            try:
                if hasattr(cv2, 'VideoWriter_fourcc'):
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
                elif hasattr(cv2, 'cv') and hasattr(cv2.cv, 'CV_FOURCC'):
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.cv.CV_FOURCC('M','J','P','G'))
            except Exception as e:
                print(f"{COLOR_YELLOW}[WARNING] 设置MJPG格式失败: {e}, 使用默认格式{COLOR_RESET}")

            # 循环尝试设置属性，有些摄像头可能需要多次设置才能生效
            max_prop_attempts = 3
            for prop_attempt in range(max_prop_attempts):
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                cap.set(cv2.CAP_PROP_FPS, fps)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

                # 验证设置是否生效
                actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                actual_fps = cap.get(cv2.CAP_PROP_FPS)

                # 检查设置是否生效（允许一定的容差）
                if (abs(actual_width - width) < width * 0.1 and 
                    abs(actual_height - height) < height * 0.1):
                    break

                if prop_attempt < max_prop_attempts - 1:
                    debug(f"属性设置未生效，重试中... (尝试 {prop_attempt+1}/{max_prop_attempts})", "frame")
                    time.sleep(0.2)

            # 读取多帧测试摄像头是否真正工作
            stable_frames = 0
            max_test_frames = 5
            for _ in range(max_test_frames):
                test_ret, test_frame = cap.read()
                if test_ret and test_frame is not None and test_frame.size > 0:
                    stable_frames += 1
                time.sleep(0.05)

            if stable_frames < 3:  # 至少要成功读取3帧才算稳定
                raise RuntimeError(f"摄像头不稳定，只能读取 {stable_frames}/{max_test_frames} 帧")

            # 更新全局状态并返回成功
            print(f"{COLOR_GREEN}[INFO] 摄像头初始化成功: 实际分辨率 {actual_width}×{actual_height}, FPS {actual_fps}{COLOR_RESET}")
            return True

        except Exception as e:
            print(f"{COLOR_YELLOW}[WARNING] 摄像头初始化尝试 {attempt+1}/{retries} 失败: {e}{COLOR_RESET}")

            # 指数退避策略
            if attempt < retries - 1:
                backoff_time = min(backoff_time * 1.5, 10.0)  # 指数增长但最多等待10秒
                print(f"{COLOR_YELLOW}[WARNING] {backoff_time:.1f}秒后重试...{COLOR_RESET}")
                time.sleep(backoff_time)
            else:
                print(f"{COLOR_RED}[ERROR] 摄像头初始化失败，已达到最大尝试次数{COLOR_RESET}")
                return False

    return False

# ───────── 车道线检测 + 角度误差 计算 ─────────
def compute_angle_error(frame: np.ndarray, roi_ratio, num_slices, valid_area, max_area, hsv_lower, hsv_upper, kernel, color_name: str) -> tuple[float, int]:
    """增强版车道线检测函数，添加异常处理和鲁棒性增强"""
    debug(f"开始 {color_name} 车道线检测", "lane")

    # 输入参数验证
    if frame is None or frame.size == 0:
        debug(f"{color_name} 车道线检测: 无效的帧", "lane")
        return 0.0, 0

    try:
        h, w = frame.shape[:2]

        # 参数验证和安全限制
        if not (0.0 < roi_ratio < 1.0):
            debug(f"{color_name} 车道线检测: ROI比例异常 ({roi_ratio})，使用默认值0.5", "lane")
            roi_ratio = 0.5

        if num_slices <= 0:
            debug(f"{color_name} 车道线检测: 切片数异常 ({num_slices})，使用默认值3", "lane")
            num_slices = 3

        # 安全计算ROI区域
        roi_y0 = max(0, min(h-1, int(h * (1 - roi_ratio))))
        slice_h = max(1, (h - roi_y0) // num_slices)  # 确保至少为1，避免除零错误

        # 使用异常处理包装图像处理操作
        try:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        except Exception as e:
            debug(f"{color_name} 车道线检测: 颜色空间转换失败 ({e})", "lane")
            return 0.0, 0

        # 确保HSV限制值是有效的，防止无效的过滤结果
        if not (isinstance(hsv_lower, np.ndarray) and isinstance(hsv_upper, np.ndarray) and
                hsv_lower.shape == (3,) and hsv_upper.shape == (3,)):
            debug(f"{color_name} 车道线检测: 无效的HSV阈值", "lane")
            # 使用默认黑色阈值
            hsv_lower = np.array([0, 0, 0], np.uint8)
            hsv_upper = np.array([180, 255, 60], np.uint8)

        # 图像掩码处理异常处理
        try:
            mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.dilate(mask, kernel)
        except Exception as e:
            debug(f"{color_name} 车道线检测: 掩码处理失败 ({e})", "lane")
            return 0.0, 0

        # 结果变量初始化
        sum_abs = 0.0
        sum_raw = 0.0
        count = 0
        angles = []  # 收集所有角度用于后处理
        weights = []  # 每个角度的权重（基于面积）

        # 切片分析
        for i in range(num_slices):
            try:
                y1 = roi_y0 + i * slice_h
                y2 = min(h, h if i == num_slices - 1 else (roi_y0 + (i + 1) * slice_h))

                if y1 >= y2 or y1 >= h or y2 <= 0:
                    debug(f"切片 {i}: 无效边界 y1={y1}, y2={y2}", "lane")
                    continue

                slice_mask = mask[y1:y2]
                if slice_mask.size == 0:
                    debug(f"切片 {i}: 掩码为空", "lane")
                    continue

                cnts, _ = cv2.findContours(slice_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if not cnts:
                    debug(f"切片 {i}: 未找到轮廓", "lane")
                    continue

                # 筛选最大轮廓，同时进行异常检测
                largest = max(cnts, key=cv2.contourArea)
                area = cv2.contourArea(largest)

                # 面积检查
                if area < valid_area or area > max_area:
                    debug(f"切片 {i}: 轮廓面积 {area} 不在有效范围内 [{valid_area}, {max_area}]", "lane")
                    continue

                # 计算中心点位置
                x, y, w_box, h_box = cv2.boundingRect(largest)
                if w_box <= 0 or h_box <= 0:
                    debug(f"切片 {i}: 无效的边界框 w={w_box}, h={h_box}", "lane")
                    continue

                cx = x + w_box // 2
                cy = y + h_box // 2 + y1

                # 计算角度，使用安全的处理防止极端情况
                dx = (w / 2.0) - float(cx)
                dy = float(h) - float(cy)

                # 防止除零错误
                if abs(dy) < 1e-6:
                    dy = 1e-6

                angle_i = np.degrees(np.arctan2(dx, dy))

                # 异常值检测 - 角度不应超过合理范围（一般车道线角度不会太大）
                if abs(angle_i) > 60.0:
                    debug(f"切片 {i}: 角度值异常 ({angle_i:.2f}°)，可能是噪声", "lane")
                    continue

                sum_abs += abs(angle_i)
                sum_raw += angle_i
                count += 1

                # 保存角度和权重
                angles.append(angle_i)
                weights.append(area)  # 使用面积作为权重

                debug(f"切片 {i}: 找到有效轮廓，面积={area}, 质心=({cx}, {cy}), 角度={angle_i:.2f}°", "lane")
            except Exception as slice_error:
                debug(f"切片 {i} 处理异常: {slice_error}", "lane")
                continue

        # 结果处理
        if count == 0:
            debug(f"{color_name} 车道线检测: 未找到有效轮廓", "lane")
            return 0.0, 0

        # 异常值过滤 - 使用中位数过滤异常角度
        if len(angles) >= 3:
            # 对角度进行排序
            sorted_idx = np.argsort(angles)
            median_idx = sorted_idx[len(sorted_idx) // 2]
            median_angle = angles[median_idx]

            # 过滤掉偏离中位数太远的角度
            filtered_angles = []
            filtered_weights = []
            for i, angle in enumerate(angles):
                if abs(angle - median_angle) <= 15.0:  # 允许的偏差范围
                    filtered_angles.append(angle)
                    filtered_weights.append(weights[i])

            if filtered_angles:
                # 使用加权平均获得最终角度
                weighted_sum = sum(a * w for a, w in zip(filtered_angles, filtered_weights))
                total_weight = sum(filtered_weights)
                avg_raw = weighted_sum / total_weight
                avg_abs = sum(abs(a) * w for a, w in zip(filtered_angles, filtered_weights)) / total_weight
                count = len(filtered_angles)
                debug(f"{color_name} 车道线检测: 过滤后保留 {count} 个有效角度", "lane")
            else:
                # 如果过滤后没有角度，则回退到普通平均
                avg_abs = sum_abs / count
                avg_raw = sum_raw / count
        else:
            # 样本数量少，使用普通平均
            avg_abs = sum_abs / count
            avg_raw = sum_raw / count

        # 确定方向
        direction = 1.0 if avg_raw > 1e-6 else (-1.0 if avg_raw < -1e-6 else 0.0)

        # 平滑处理结果
        final_angle = avg_abs * direction
        debug(f"{color_name} 车道线检测: 找到 {count} 个有效轮廓, 平均角度={avg_raw:.2f}°, 方向={direction}", "lane")
        return final_angle, count

    except Exception as e:
        debug(f"{color_name} 车道线检测: 未处理异常 {e}", "lane")
        return 0.0, 0

# ───────── 退出时 发送零速度 并 清理 ─────────
def send_zero_and_cleanup():
    """增强版资源清理函数，提供更强的容错性和资源释放确认"""
    global ser, cap, ERROR_COUNT

    print(f"
{COLOR_GREEN}[INFO] 开始安全停车和资源清理...{COLOR_RESET}")

    # 状态跟踪变量
    success_serial = False
    success_camera = False

    # 1. 发送零速度指令 - 重试机制增强
    for i in range(5):  # 增加重试次数，确保安全停车
        try:
            if ser is not None:
                if not ser.is_open:
                    try:
                        print(f"{COLOR_YELLOW}[WARNING] 串口已关闭，尝试重新打开...{COLOR_RESET}")
                        ser.open()
                        time.sleep(0.1)
                    except Exception as open_error:
                        print(f"{COLOR_YELLOW}[WARNING] 重新打开串口失败: {open_error}{COLOR_RESET}")
                        continue

                if ser.is_open:
                    print(f"{COLOR_GREEN}[INFO] 发送零速度帧 (尝试 {i+1}/5)...{COLOR_RESET}")
                    # 使用不同方式构建零速度帧，提高可靠性
                    if i < 3:
                        # 使用标准方法
                        zero_frame = build_frame(0.0, 0.0, 0.0)
                    else:
                        # 使用备用方法 - 手动创建帧
                        print(f"{COLOR_YELLOW}[WARNING] 使用备用方法构建零速度帧{COLOR_RESET}")
                        zero_frame = bytes([FRAME_HEADER, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, FRAME_TAIL])

                    # 发送帧
                    bytes_written = ser.write(zero_frame)
                    ser.flush()  # 确保数据发送完成

                    # 验证写入长度
                    if bytes_written == len(zero_frame):
                        print(f"{COLOR_GREEN}[INFO] 零速度帧发送成功 ({bytes_written} 字节){COLOR_RESET}")
                        success_serial = True
                        break
                    else:
                        print(f"{COLOR_YELLOW}[WARNING] 零速度帧发送不完整: {bytes_written}/{len(zero_frame)} 字节{COLOR_RESET}")

                    time.sleep(0.2)  # 延长等待时间，确保命令被处理
        except Exception as e:
            print(f"{COLOR_YELLOW}[WARNING] 发送零速度帧失败 (尝试 {i+1}/5): {e}{COLOR_RESET}")
            time.sleep(0.5)  # 增加重试间隔

    if not success_serial:
        print(f"{COLOR_RED}[ERROR] 所有零速度帧发送尝试均失败{COLOR_RESET}")

    # 2. 清理摄像头资源 - 强化清理过程
    camera_release_attempts = 0
    while camera_release_attempts < 3 and not success_camera:
        try:
            if cap is not None:
                print(f"{COLOR_GREEN}[INFO] 释放摄像头资源 (尝试 {camera_release_attempts+1}/3)...{COLOR_RESET}")

                # 在Windows系统上可能需要特殊处理
                if os.name == 'nt':
                    # Windows: 尝试特定的清理方式
                    try:
                        cap.release()
                        # 在Windows上验证释放状态
                        if hasattr(cap, 'isOpened') and not cap.isOpened():
                            success_camera = True
                    except Exception as e:
                        print(f"{COLOR_YELLOW}[WARNING] Windows摄像头释放异常: {e}{COLOR_RESET}")
                else:
                    # Linux/其他: 标准清理
                    cap.release()
                    success_camera = True

                if success_camera:
                    print(f"{COLOR_GREEN}[INFO] 摄像头资源已成功释放{COLOR_RESET}")
                    cap = None  # 明确置空引用
                else:
                    print(f"{COLOR_YELLOW}[WARNING] 摄像头可能未完全释放{COLOR_RESET}")
        except Exception as e:
            print(f"{COLOR_YELLOW}[WARNING] 释放摄像头失败 (尝试 {camera_release_attempts+1}/3): {e}{COLOR_RESET}")

        camera_release_attempts += 1
        if not success_camera and camera_release_attempts < 3:
            print(f"{COLOR_YELLOW}[WARNING] 0.5秒后重试释放摄像头...{COLOR_RESET}")
            time.sleep(0.5)

    if not success_camera:
        print(f"{COLOR_RED}[ERROR] 所有摄像头释放尝试均失败{COLOR_RESET}")
        cap = None  # 强制清除引用，让垃圾收集器处理

    # 3. 清理串口资源 - 强化清理过程
    serial_close_attempts = 0
    success_serial_close = False

    while serial_close_attempts < 3 and not success_serial_close:
        try:
            if ser is not None:
                if ser.is_open:
                    print(f"{COLOR_GREEN}[INFO] 关闭串口 (尝试 {serial_close_attempts+1}/3)...{COLOR_RESET}")
                    ser.close()

                    # 验证串口是否真的关闭了
                    if not ser.is_open:
                        print(f"{COLOR_GREEN}[INFO] 串口已成功关闭{COLOR_RESET}")
                        success_serial_close = True
                    else:
                        print(f"{COLOR_YELLOW}[WARNING] 串口关闭调用后仍处于打开状态{COLOR_RESET}")
                else:
                    print(f"{COLOR_GREEN}[INFO] 串口已经处于关闭状态{COLOR_RESET}")
                    success_serial_close = True
        except Exception as e:
            print(f"{COLOR_YELLOW}[WARNING] 关闭串口失败 (尝试 {serial_close_attempts+1}/3): {e}{COLOR_RESET}")

        serial_close_attempts += 1
        if not success_serial_close and serial_close_attempts < 3:
            print(f"{COLOR_YELLOW}[WARNING] 0.5秒后重试关闭串口...{COLOR_RESET}")
            time.sleep(0.5)

    # 4. 重置错误计数和其他状态变量
    ERROR_COUNT = 0

    # 5. 最终清理状态报告
    status_ok = success_camera and success_serial_close
    if status_ok:
        print(f"{COLOR_GREEN}[INFO] 资源清理成功完成{COLOR_RESET}")
    else:
        issue_list = []
        if not success_camera:
            issue_list.append("摄像头未释放")
        if not success_serial_close:
            issue_list.append("串口未关闭")
        print(f"{COLOR_YELLOW}[WARNING] 资源清理部分完成，存在问题: {', '.join(issue_list)}{COLOR_RESET}")

    return status_ok

def signal_handler(sig, frame):
    print(f"\n{COLOR_YELLOW}接收到终止信号 {sig}, 正在清理资源...{COLOR_RESET}")
    send_zero_and_cleanup()
    sys.exit(0)

# ───────── 检测二维码 ─────────
def detect_qr_code(frame):
    """增强版二维码检测，添加异常处理和多种检测策略"""
    debug("开始二维码检测...", "qr")

    if frame is None or frame.size == 0:
        debug("二维码检测失败: 帧为空", "qr")
        return False, None

    try:
        # 尝试不同的图像处理方法以提高检测率
        success = False
        content = None

        # 1. 原始灰度图检测
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            decoded_objects = pyzbar.decode(gray)
            if len(decoded_objects) > 0:
                for obj in decoded_objects:
                    try:
                        content = obj.data.decode("utf-8")
                        debug(f"灰度图检测到二维码: {content}", "qr")
                        return True, content
                    except UnicodeDecodeError:
                        debug("二维码解码失败: 非UTF-8编码", "qr")
                        try:
                            # 尝试其他编码方式
                            content = obj.data.decode("iso-8859-1")
                            debug(f"使用ISO-8859-1编码成功解码: {content}", "qr")
                            return True, content
                        except Exception:
                            pass
        except Exception as e:
            debug(f"灰度图二维码检测异常: {e}", "qr")

        # 2. 尝试增强对比度
        try:
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            enhanced_gray = clahe.apply(gray)
            decoded_objects = pyzbar.decode(enhanced_gray)
            if len(decoded_objects) > 0:
                for obj in decoded_objects:
                    try:
                        content = obj.data.decode("utf-8")
                        debug(f"增强对比度后检测到二维码: {content}", "qr")
                        return True, content
                    except Exception:
                        pass
        except Exception as e:
            debug(f"增强对比度二维码检测异常: {e}", "qr")

        # 3. 尝试调整阈值
        try:
            _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
            decoded_objects = pyzbar.decode(thresh)
            if len(decoded_objects) > 0:
                for obj in decoded_objects:
                    try:
                        content = obj.data.decode("utf-8")
                        debug(f"二值化后检测到二维码: {content}", "qr")
                        return True, content
                    except Exception:
                        pass
        except Exception as e:
            debug(f"二值化二维码检测异常: {e}", "qr")

        # 4. 尝试缩放图像以适应不同大小的二维码
        try:
            h, w = frame.shape[:2]
            # 缩放系数，尝试不同大小
            for scale in [0.5, 0.75, 1.5, 2.0]:
                scaled_w, scaled_h = int(w * scale), int(h * scale)
                if scaled_w <= 0 or scaled_h <= 0:
                    continue

                resized = cv2.resize(frame, (scaled_w, scaled_h))
                gray_resized = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
                decoded_objects = pyzbar.decode(gray_resized)
                if len(decoded_objects) > 0:
                    for obj in decoded_objects:
                        try:
                            content = obj.data.decode("utf-8")
                            debug(f"缩放({scale}x)后检测到二维码: {content}", "qr")
                            return True, content
                        except Exception:
                            pass
        except Exception as e:
            debug(f"缩放图像二维码检测异常: {e}", "qr")

        debug("尝试多种方法后仍未检测到二维码", "qr")
        return False, None

    except Exception as e:
        debug(f"二维码检测过程中发生未处理的异常: {e}", "qr")
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
    global cap, ser, pid, red_pid, VX_CONST, VALID_AREA, MAX_AREA, DEBUG, ERROR_COUNT, CAR_RUNNING
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
            try:
                ok, frame = cap.read() if hasattr(cap, 'read') else (False, None)
                if not ok or frame is None:
                    print(f"{COLOR_YELLOW}[WARNING] 读取帧失败，跳过当前帧{COLOR_RESET}")
                    ERROR_COUNT += 1

                    # 如果连续多次读取失败，尝试重新初始化摄像头
                    if ERROR_COUNT > ERROR_THRESHOLD:
                        print(f"{COLOR_YELLOW}[WARNING] 连续多次读取失败，尝试重新初始化摄像头{COLOR_RESET}")
                        init_camera(args.cam_device, args.cam_width, args.cam_height, args.cam_fps)
                        ERROR_COUNT = 0

                    time.sleep(0.1)
                    continue
                else:
                    # 读取成功，重置错误计数
                    ERROR_COUNT = max(0, ERROR_COUNT - 1)
            except Exception as e:
                print(f"{COLOR_YELLOW}[WARNING] 读取帧异常: {e}, 跳过当前帧{COLOR_RESET}")
                ERROR_COUNT += 1
                time.sleep(0.1)
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
            # 安全发送控制指令
            if ser is not None:
                try:
                    frame_to_send = build_frame(send_vx, vy, send_vz)

                    # 检查串口状态
                    if not ser.is_open:
                        print(f"{COLOR_YELLOW}[WARNING] 串口未打开，尝试重新打开{COLOR_RESET}")
                        try:
                            ser.open()
                            time.sleep(0.1)
                        except Exception as e:
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
                            print(f"{COLOR_RED}[ERROR] 串口发送异常: {e}{COLOR_RESET}")
                            ERROR_COUNT += 1

                        if success:
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
                                print(f"{COLOR_RED}[ERROR] 重置串口连接失败: {e}{COLOR_RESET}")
                except Exception as e:
                    print(f"{COLOR_RED}[ERROR] 串口通信处理异常: {e}{COLOR_RESET}")
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