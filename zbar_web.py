#!/usr/bin/env python3
"""
MiniCar 摄像头展示（Flask + OpenCV + QR 识别）
----------------------------------------------
⚡ 低 CPU 版 & QR 识别（线程池）：
* 使用 **pyzbar** 对 *灰度* 帧做二维码检测，再在彩色帧上画框/文字。
* 采用 **ThreadPoolExecutor**（线程池）执行解码，减少主线程阻塞；推流 FPS 与识别 FPS 仍保持一致（处理期间缓冲帧被丢弃）。
* 采集分辨率仍尝试 1280×720 MJPG；失败时自动缩放并灰边填充。
依赖：Flask, OpenCV‑Python, NumPy, pyzbar
"""

import os
import time
import cv2
import numpy as np
import concurrent.futures as futures
from flask import Flask, Response, render_template_string, jsonify
from pyzbar import pyzbar

# ──────────────── 配置 ────────────────
DEVICE         = int(os.getenv("CAM_DEVICE", 0))
TARGET_W       = int(os.getenv("CAM_WIDTH", 1280))
TARGET_H       = int(os.getenv("CAM_HEIGHT", 720))
TARGET_FPS     = int(os.getenv("CAM_FPS", 30))      # 摄像头尝试帧率
JPEG_QUALITY   = int(os.getenv("JPEG_QUALITY", 70)) # 1‑100
FOURCC         = "MJPG"
QR_THREADS     = int(os.getenv("QR_THREADS", 2))    # 线程池大小

# ──────────────── 初始化摄像头 ────────────────
cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
if not cap.isOpened():
    raise RuntimeError("无法打开摄像头")

cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*FOURCC))
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  TARGET_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TARGET_H)
cap.set(cv2.CAP_PROP_FPS,         TARGET_FPS)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 仅保留最新帧，减少延迟

src_w  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
src_h  = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
src_fps = cap.get(cv2.CAP_PROP_FPS) or 0
print(f"Camera native {src_w}×{src_h}  {src_fps:.1f} FPS")

# ──────────────── 线程池 ────────────────
executor = futures.ThreadPoolExecutor(max_workers=QR_THREADS)

# ──────────────── 工具函数 ────────────────
def fit_to_canvas(frame: np.ndarray) -> np.ndarray:
    """将任意分辨率 frame 缩放并居中填充到 TARGET_W×TARGET_H 画布"""
    h, w = frame.shape[:2]
    scale = min(TARGET_W / w, TARGET_H / h)
    new_w, new_h = int(w * scale), int(h * scale)
    if scale != 1.0:
        interp = cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR
        frame = cv2.resize(frame, (new_w, new_h), interpolation=interp)
    canvas = np.full((TARGET_H, TARGET_W, 3), 128, np.uint8)
    x0, y0 = (TARGET_W - new_w)//2, (TARGET_H - new_h)//2
    canvas[y0:y0+new_h, x0:x0+new_w] = frame
    return canvas

def draw_decoded(frame: np.ndarray, decoded):
    """在 frame 上绘制解码结果"""
    for obj in decoded:
        x, y, w, h = obj.rect
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        txt = obj.data.decode("utf-8", "ignore")
        cv2.putText(frame, txt, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 0), 2, cv2.LINE_AA)

# ──────────────── 统计数据 ────────────────
STATS = {
    "cam_fps":  src_fps,
    "proc_fps": 0.0,   # 识别/推流 FPS
    "latency_ms": 0.0,
    "cam_resolution": f"{src_w}×{src_h}"
}
_cnt, _t0 = 0, time.time()

# ──────────────── Flask 应用 ────────────────
app = Flask(__name__)


def gen_frames():
    """抓取摄像头帧 → 线程池灰度解码 → 彩色绘制 → MJPEG 推流"""
    global _cnt, _t0
    params = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]

    while True:
        t_cap = time.time()
        ok, color = cap.read()
        if not ok:
            continue

        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        # 在线程池中解码（同步等待结果）
        decoded = executor.submit(pyzbar.decode, gray).result()
        if decoded:
            draw_decoded(color, decoded)

        # 缩放/填充到目标大小
        if (color.shape[1], color.shape[0]) != (TARGET_W, TARGET_H):
            color = fit_to_canvas(color)

        STATS["latency_ms"] = (time.time() - t_cap) * 1000

        _cnt += 1
        if time.time() - _t0 >= 1:
            STATS["proc_fps"] = _cnt / (time.time() - _t0)
            _cnt, _t0 = 0, time.time()

        ok, buf = cv2.imencode('.jpg', color, params)
        if not ok:
            continue
        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n'


@app.route('/')
def index():
    html = """<!doctype html><html lang='zh-CN'><head><meta charset='utf-8'>
    <title>MiniCar 摄像头展示 (QR)</title>
    <style>
      html,body{height:100%;margin:0;overflow:hidden;background:#666;color:#fff;display:flex;flex-direction:column;font-family:-apple-system,Segoe UI,Roboto,Helvetica,Arial,sans-serif}
      .title{background:#FFD700;color:#000;text-align:center;font-size:1.4rem;padding:0.6rem 0;font-weight:normal;flex-shrink:0}
      .view{flex:1 1 auto;display:flex;align-items:center;justify-content:center}
      .view img{max-width:100%;max-height:100%;object-fit:contain}
      footer{position:fixed;bottom:0;left:0;width:100%;background:#444;color:#eee;padding:4px 0;font-size:0.9rem;text-align:center}
      footer span{color:#0f0;margin:0 0.3rem}
    </style></head><body>
      <div class='title'>MiniCar 摄像头展示 (QR 识别)</div>
      <div class='view'><img src='{{ url_for("video_feed") }}' alt='camera'></div>
      <footer>
        摄像头 FPS:<span id='cam'>--</span> ｜ 识别/推流 FPS:<span id='proc'>--</span>
        ｜ 延迟:<span id='lat'>--</span>ms ｜ 原始分辨率:<span id='res'>--</span>
      </footer>
      <script>
        async function poll() {
          try {
            const d = await (await fetch('/stats')).json();
            document.getElementById('cam').textContent  = d.cam_fps.toFixed(1);
            document.getElementById('proc').textContent = d.proc_fps.toFixed(1);
            document.getElementById('lat').textContent  = d.latency_ms.toFixed(1);
            document.getElementById('res').textContent  = d.cam_resolution;
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


if __name__ == '__main__':
    try:
        app.run('0.0.0.0', 8080, threaded=True)
    finally:
        cap.release()
        executor.shutdown(wait=False)
