#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MiniCar 摄像头展示 + 车道线可视化（Flask + OpenCV）
------------------------------------------------------
• 每个 ROI 切片只保留面积最大的白色连通域，面积 ≥ VALID_AREA 才绘制
• 显示蓝色矩形、红色中心点，以及黄色切片分割线和 ROI 顶线
"""

import os, time, cv2, numpy as np
from flask import Flask, Response, render_template_string, jsonify

# ───────── 摄像头 & 推流参数 ─────────
DEVICE        = os.getenv("CAM_DEVICE", 0)
TARGET_W      = int(os.getenv("CAM_WIDTH" , 1280))
TARGET_H      = int(os.getenv("CAM_HEIGHT", 720))
TARGET_FPS    = int(os.getenv("CAM_FPS"  , 30))
STREAM_FPS    = int(os.getenv("STREAM_FPS", 15))
JPEG_QUALITY  = int(os.getenv("JPEG_QUALITY", 70))
FRAME_INTERVAL = 1.0 / STREAM_FPS
FOURCC = "MJPG"

# ───────── 车道线算法参数 ─────────
ROI_RATIO  = float(os.getenv("LANE_ROI_RATIO", 0.5))   # 0.5 = 下半幅
NUM_SLICES = int(os.getenv("LANE_SLICES", 5))
VALID_AREA = int(os.getenv("LANE_MIN_AREA", 3000))      # 最小有效面积
VALID_AREA = int(os.getenv("LANE_MAX_AREA", 10000))  # 确保至少有一个像素

HSV_LOWER = np.array([0, 0, 0],   np.uint8)
HSV_UPPER = np.array([180, 255, 100], np.uint8)
KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

COLOR_LINE, COLOR_RECT, COLOR_POINT = (0,255,255), (255,0,0), (0,0,255)
THICK_LINE, POINT_R = 2, 6

# ───────── 初始化摄像头 ─────────
cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
if not cap.isOpened():
    raise RuntimeError("无法打开摄像头")
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*FOURCC))
cap.set(cv2.CAP_PROP_FRAME_WIDTH , TARGET_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TARGET_H)
cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

src_w, src_h = int(cap.get(3)), int(cap.get(4))
src_fps = cap.get(cv2.CAP_PROP_FPS) or 0
print(f"Camera native {src_w}×{src_h}  {src_fps:.1f} FPS")

# ───────── utils ─────────
def fit_to_canvas(img):
    h, w = img.shape[:2]
    s = min(TARGET_W / w, TARGET_H / h)
    nw, nh = int(w * s), int(h * s)
    if s != 1:
        img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_LINEAR)
    canvas = np.full((TARGET_H, TARGET_W, 3), 128, np.uint8)
    x0, y0 = (TARGET_W - nw) // 2, (TARGET_H - nh) // 2
    canvas[y0:y0+nh, x0:x0+nw] = img
    return canvas

# ───────── 车道线可视化 ─────────
def visualise_lane(frame):
    h, w = frame.shape[:2]
    roi_y0 = int(h * (1 - ROI_RATIO))
    slice_h = (h - roi_y0) // NUM_SLICES

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, 1)
    mask = cv2.dilate(mask, KERNEL, 1)

    for i in range(NUM_SLICES):
        y1 = roi_y0 + i * slice_h
        y2 = h if i == NUM_SLICES - 1 else roi_y0 + (i + 1) * slice_h
        slice_mask = mask[y1:y2]

        cnts, _ = cv2.findContours(slice_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            continue
        largest = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(largest) < VALID_AREA:
            continue

        x, y, w_box, h_box = cv2.boundingRect(largest)
        cv2.rectangle(frame, (x, y + y1), (x + w_box, y + y1 + h_box), COLOR_RECT, THICK_LINE)
        cx, cy = x + w_box // 2, y + h_box // 2 + y1
        cv2.circle(frame, (cx, cy), POINT_R, COLOR_POINT, -1)

    # ==== 分割线 ====
    for i in range(NUM_SLICES):
        y_split = roi_y0 + i * slice_h
        cv2.line(frame, (0, y_split), (w, y_split), COLOR_LINE, THICK_LINE)
    cv2.line(frame, (0, roi_y0), (w, roi_y0), COLOR_LINE, THICK_LINE)

    return frame

# ───────── 统计 ─────────
STATS = {"ideal_fps": TARGET_FPS, "cam_fps": src_fps,
         "web_fps": 0.0, "latency_ms": 0.0, "cam_resolution": f"{src_w}×{src_h}"}
_cnt, _t0, _last = 0, time.time(), 0

# ───────── Flask ─────────
app = Flask(__name__)

def gen_frames():
    global _cnt, _t0, _last
    params = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
    while True:
        dt = time.time() - _last
        if dt < FRAME_INTERVAL:
            time.sleep(FRAME_INTERVAL - dt)
        _last = time.time()

        tic = time.time()
        ok, frame = cap.read()
        if not ok:
            continue
        frame = visualise_lane(frame)
        frame = frame if (frame.shape[1], frame.shape[0]) == (TARGET_W, TARGET_H) else fit_to_canvas(frame)

        STATS["latency_ms"] = (time.time() - tic) * 1000
        _cnt += 1
        if time.time() - _t0 >= 1:
            STATS["web_fps"] = _cnt / (time.time() - _t0)
            _cnt = 0
            _t0 = time.time()

        ok, buf = cv2.imencode('.jpg', frame, params)
        if not ok:
            continue
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' +
               buf.tobytes() + b'\r\n')

@app.route('/')
def index():
    return render_template_string("""<!doctype html><html><head><meta charset='utf-8'>
    <title>MiniCar ‑ Lane Visual</title><style>
    html,body{height:100%;margin:0;background:#666;color:#fff;display:flex;flex-direction:column;font-family:sans-serif}
    .title{background:#FFD700;color:#000;text-align:center;padding:.6rem 0}
    .view{flex:1;display:flex;align-items:center;justify-content:center}
    .view img{max-width:100%;max-height:100%;object-fit:contain}
    footer{position:fixed;bottom:0;left:0;width:100%;background:#444;font-size:.9rem;text-align:center;padding:4px 0}
    footer span{color:#0f0;margin:0 .3rem}</style></head><body>
    <div class='title'>MiniCar 摄像头 – 车道线可视化</div>
    <div class='view'><img src='{{ url_for("video_feed") }}'></div>
    <footer>理想:<span id=i>--</span>fps│相机:<span id=c>--</span>fps│Web:<span id=w>--</span>fps│延迟:<span id=l>--</span>ms│分辨率:<span id=r>--</span></footer>
    <script>
    async function p(){try{const d=await (await fetch('/stats')).json();
    i.textContent=d.ideal_fps.toFixed(1);c.textContent=d.cam_fps.toFixed(1);
    w.textContent=d.web_fps.toFixed(1);l.textContent=d.latency_ms.toFixed(1);
    r.textContent=d.cam_resolution;}catch(e){}setTimeout(p,1000);}p();
    </script></body></html>""")

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
