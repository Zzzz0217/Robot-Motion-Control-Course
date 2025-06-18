# Robot-Motion-Control-Course
This is a repository for Robot Motion Control Experiment Course of Faculty of Robot Science and Engineering of Northeastern University. Especially owned by student Jason Z.
## Hardware
We use a car of the WHEELTEC, which uses STM32 as its lower computer, while uses Raspberry Pi 4 as its upper computer. This car is an Ackerman car, uses two motors and two servos.
## Software
This repository basically use Python as coding language to send command to control the car. In this repository there are several Python file:
### Basic ability
In file "main.py", we use traditional computer vision method to pick up the black race line and process the line to make it smooth and clear; Then we send a 'vx_base' to the car to run it, while following the black race line center which we calculated by the line we picked up before; For the line center, we choose the lower half piece of the image and cut it into five pieces, and calculate center point of each piece, and then calculate their average point; We use PID algorithm to control the car, use both angle_PID and speed_PID to make it run smmother in curve; And we have a strategy to process the condition if the car failed to track the line: If the car failed to track the line, it uses the max angle and a certain speed to trace back to the line according to the last condition of the servo to decide left or right angle.
### Advanced ability
In file "scan.py", we achieve the ability of scanning the QR Code and stop the car. Base on the main.py strategy, we added a process of the red line(red line for lower the speed and PID so that the camera can read QR Code in time), and start to scan the QR Code, if scanned, send 0 speed to the car. To make the car run correctly on the black and red line, we apply two different PID parameters in which one of them is more radical, while the other is more conservative.
## How TO Use ?
To use this car, use ssh to connect the Raspberry Pi(ur computer and the car should connect to the same WIFI), the host is now root@192.168.162.38, password: 6.