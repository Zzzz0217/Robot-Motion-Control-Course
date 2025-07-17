```plaintext
# Robot-Motion-Control-Course

## 项目概述
本项目是东北大学机器人科学与工程学院机器人运动控制实验课程的仓库。项目主要围绕 MiniCar 展开，运用 Python 进行编程，实现小车的运动控制、车道线检测、二维码识别等功能，并通过 Flask 搭建 Web 界面进行实时展示。

## 软硬件信息

### 硬件
- **小车**：采用 WHEELTEC 轮趣科技的小车，以 STM32F407 为主控制器，树莓派 4b 作为上位机。该小车为阿克曼车辆，配备两个电机和两个舵机。配备一个RGB摄像头，用于图像采集。

### 软件
项目主要使用 Python 语言编写，包含多个 Python 文件，各文件功能如下：
- `main.py`：运用传统计算机视觉方法检测黑色赛道线，实现角度环和速度环的 PID 控制，根据车道线位置调整小车的行驶速度和方向。同时可选web界面展示摄像头画面和统计数据，通过 Flask 搭建网页展示图像和二维码识别结果。
- `line_code.py`：实现 MiniCar 的角度 PID 巡线，支持黑色和红色双线识别，具备独立的 PID 控制器和保守策略，支持二维码扫描，扫描成功后停车。
- `zbar_web.py`：使用树莓派摄像头采集图像，通过 OpenCV 处理图像进行二维码扫描，利用 Flask 搭建网页展示图像和二维码识别结果。
- `line_web.py`：同样从树莓派摄像头采集图像，经 OpenCV 处理检测赛道线，将数据传回计算机，通过 Flask 在网页上展示图像。

## 功能特性

### 基本功能
- **车道线检测**：在 `main.py` 和 `line_code.py` 中，通过传统计算机视觉方法对图像进行处理，检测黑色和红色车道线。
- **PID 控制**：实现角度环和速度环的 PID 控制，根据车道线的位置和角度误差调整小车的行驶速度和方向。
- **速度调节**：根据车道线的角度调整小车的行驶速度，弯道减速，直道快速通过。

### 高级功能
- **双线识别**：在 `line_code.py` 中支持黑色和红色双线识别，针对不同颜色的车道线应用不同的 PID 参数。
- **二维码扫描**：在 `line_code.py` 和 `zbar_web.py` 中支持二维码扫描，扫描成功后小车停车。
- **Web 展示**：通过 `zbar_web.py` 和 `line_web.py` 搭建 Web 界面，实时展示摄像头画面和统计数据。

## 安装与配置

### 所需软件包
在运行项目之前，需要安装以下 Python 包：
```plaintext
- opencv-python
- numpy
- serial
- zbarlight
- flask
- time
- threading
- math
- cv2
- sys
- os
- subprocess
- re
- json
- requests
- datetime
- random
- argparse
- imutils
- pyzbar
```
你可以使用以下命令来安装这些包：
```bash
pip install opencv-python numpy serial zbarlight flask pyzbar imutils
```

### 配置参数
部分参数可通过环境变量进行配置，以下是一些主要参数及其默认值：
```plaintext
- CAM_DEVICE: 摄像头设备编号，默认值为 0
- CAM_WIDTH: 摄像头采集宽度，默认值为 1280
- CAM_HEIGHT: 摄像头采集高度，默认值为 720
- CAM_FPS: 摄像头帧率，默认值为 30
- CONTROL_FPS: 控制帧率，默认值为 15
```

## 使用方法

### 小车操作
要操作小车，需通过 SSH 连接到树莓派（你的计算机和小车应连接到同一 Wi-Fi）。当前主机为 `root@192.168.223.38`，密码为 `6`。使用以下命令进行连接：
```bash
ssh root@192.168.223.38（请根据自己的网络情况更改IP）
```
连接成功后，运行巡线程序：
```bash
python line_code.py
```

### Web 展示
要启动 Web 展示界面，运行以下命令：
```bash
python zbar_web.py
```
在浏览器中访问 `http://0.0.0.0:8080`，即可查看摄像头画面和统计数据。

## 代码结构
```plaintext
Robot-Motion-Control-Course/
├── LICENSE                 # 项目许可证
├── README.md               # 项目说明文档
├── main.py                 # 基本车道线检测和 PID 控制程序
├── line_code.py            # 角度 PID 巡线程序
├── zbar_web.py             # 摄像头展示与二维码识别 Web 程序
├── line_web.py             # 摄像头展示与车道线可视化 Web 程序
```

## 注意事项
- 确保你的计算机和小车连接到同一 Wi-Fi 网络。
- 在运行程序之前，检查摄像头是否正常工作。
- 如有需要，可根据实际情况调整配置参数。

## 许可证
本项目采用 MIT 许可证，详情请参阅 [LICENSE](LICENSE) 文件。

## 作者信息
版权所有 (c) 2025 Jason 钟晔

如有任何问题或建议，请随时联系作者。
```