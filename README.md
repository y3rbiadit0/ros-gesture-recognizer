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
```bash
sudo apt install -y python3-pip python3-virtualenv
sudo apt install libcap-dev
sudo apt install -y libcamera-apps libcamera-dev
sudo apt install python3-opencv
pip3 install picamera2 # picamera2
pip3 install mediapipe --break-system-packages # mediapipe
```

-> Install pip install picamera2 -> https://raspberrytips.com/picamera2-raspberry-pi/
-> Install opencv -> https://randomnerdtutorials.com/install-opencv-raspberry-pi/
-> Install mediapipe -> https://randomnerdtutorials.com/install-mediapipe-raspberry-pi/






### Test Code for setup
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



### Raspberry Pi OS 32 Bits - Debian Buster

-> https://github.com/Qengineering/Install-OpenCV-Raspberry-Pi-32-bits/
-> https://medium.com/@jackjin_42/raspberry-pi-4-setup-with-opencv-and-rpi-camera-1e21d789b59b

### CMake for building OpenCV from scratch config fixed - Disable TBB (intel thread building block library)
-> [Script to install OpenCV](https://github.com/Qengineering/Install-OpenCV-Raspberry-Pi-32-bits/)
### Modification to last step of script when running cmake

```bash
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules -D ENABLE_NEON=ON -D WITH_OPENMP=ON -D WITH_OPENCL=OFF -D BUILD_TIFF=ON -D WITH_FFMPEG=ON -D WITH_TBB=OFF -D BUILD_TBB=OFF -D WITH_GSTREAMER=ON -D BUILD_TESTS=OFF -D WITH_EIGEN=OFF -D WITH_V4L=ON -D WITH_LIBV4L=ON -D WITH_VTK=OFF -D WITH_QT=ON -D WITH_PROTOBUF=OFF -D OPENCV_ENABLE_NONFREE=ON -D INSTALL_C_EXAMPLES=OFF -D INSTALL_PYTHON_EXAMPLES=OFF -D OPENCV_FORCE_LIBATOMIC_COMPILER_CHECK=1 -D PYTHON3_PACKAGES_PATH=/usr/lib/python3/dist-packages -D OPENCV_GENERATE_PKGCONFIG=ON -D BUILD_EXAMPLES=OFF ..

make -j4
sudo make install
sudo apt update
```

pip3 install "picamera[array]"


### Compile ROS Considerations:
[Install ROS Noetic Steps](https://varhowto.com/install-ros-noetic-raspberry-pi-4/)

-- Modification to the step 6
```bash
# Add sensor_msgs and cv_bridge dependencies besides core ones
rosinstall_generator ros_comm sensor_msgs cv_bridge --rosdistro noetic --deps --wet-only --tar > noetic-ros_comm-wet.rosinstall

```



Run publisher
```bash
export ROS_MASTER_URI=http://<ip_where_roscore_is_being_run>:11311/
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1
python3 raspi_cam/scripts/publisher.py
```




