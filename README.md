## Robot-Motion-Control-Course
This is a repository for the Robot Motion Control Experiment Course of the Faculty of Robot Science and Engineering of Northeastern University. Especially owned by student Jason Z.

### Hardware
We use a car from WHEELTEC, which uses STM32 as its main controller, while Raspberry Pi 4 serves as its upper computer. This car is an Ackermann vehicle, equipped with two motors and two servos.

### Software
This repository basically uses Python as the coding language to send commands controlling the car. In this repository, there are several Python files:

### Basic ability
In the file "main.py", we use traditional computer vision methods to detect the black race line and process the line to make it smooth and clear. Then, we send a 'vx_base' command to the car to make it move, following the center of the detected black race line. To find the line center, we select the lower half of the image, divide it into five sections, calculate the center point of each section, and then compute their average point. We use a PID algorithm to control the car, with both angle_PID and speed_PID to make the motion smoother in curves. Additionally, we have a strategy to handle cases when the car fails to track the line: if tracking fails, the car uses the maximum steering angle and a certain speed to trace back to the line, based on the last servo condition to decide turning left or right.

### Advanced ability
In the file "scan.py", we achieve QR code scanning and stopping the car. Based on the strategy in main.py, we add a process for the red line (which indicates reducing speed and PID adjustments so the camera can read QR codes in time). When a QR code is detected, the car receives a command of zero speed to stop. To ensure correct operation on black and red lines, we apply two different sets of PID parameters—one more aggressive, the other more conservative.

### Visual Viewing On Web
In "zbar_web.py", we use the Raspberry Pi camera to capture images and send them to the computer, then use OpenCV to process the images for QR code scanning. Flask builds a web page to display the image and QR code results, shown at the top-left corner of the scan square. The web page runs on the computer, with the camera on the Raspberry Pi.
In "line_web.py", we similarly capture images from the Raspberry Pi camera, process them with OpenCV to detect the racing line, and send the data back to the computer. Flask then displays the image on a web page.

### How To Use
To operate the car, connect to the Raspberry Pi via SSH (your computer and the car should be on the same Wi-Fi). The current host is root@192.168.162.38, password: 6.

## Required Packages
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