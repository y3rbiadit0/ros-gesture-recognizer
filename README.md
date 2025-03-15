## Ros Gesture Recognizer

# Building the project

### Created `gesture_recognizer` package
```bash
catkin_create_pkg gesture_recognizer image_transport cv_bridge sensor_msgs rospy roscpp std_msgs
```

# Run Project

## 1. Install Python Dependencies

```bash
pip3 install opencv-python
pip3 install mediapipe
pip3 install tensorflow
```


## 2. Setting Up - Raspberry Pi 4 + Camera V2.1
No virtual environemnt
- Install libcap-dev
- sudo apt install -y libcamera-apps libcamera-dev
- sudo apt install python3-opencv
- pip3 install picamera2
- 

- Install pip install picamera2 -> https://raspberrytips.com/picamera2-raspberry-pi/
-> Install opencv -> https://randomnerdtutorials.com/install-opencv-raspberry-pi/
-> Install mediapipe -> https://randomnerdtutorials.com/install-mediapipe-raspberry-pi/

```python
# Minimum code to obtain the image from picamera and port it to open cv to be processed.
import cv2
import numpy as np
from picamera2 import Picamera2

cam = Picamera2()
height = 480
width = 640
middle = (int(width / 2), int(height / 2))
cam.configure(cam.create_video_configuration(main={"format": 'RGB888', "size": (width, height)}))

cam.start()

while True:
        frame = cam.capture_array()
        cv2.circle(frame, middle, 10, (255, 0 , 255), -1)
        cv2.imshow('f', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
                break
```



