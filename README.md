# ROS Gesture Recognizer
<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/2514fd23-ae21-45ad-9dd6-7258457526db" width="150"></td>
    <td><img src="https://github.com/user-attachments/assets/aa0cfd1a-4ef1-4718-b75f-624cbcd05766" width="150"></td>
    <td><img src="https://github.com/user-attachments/assets/7eaf8c0c-f8be-42c1-85bd-49bc09279640" width="150"></td>
    <td><img src="https://github.com/user-attachments/assets/71bff446-5cf0-4c38-b733-067348f047fe" width="150"></td>
  </tr>
  <tr>
    <td align="center">Raspberry Pi</td>
    <td align="center">Mediapipe</td>
    <td align="center">OpenCV</td>
    <td align="center">ROS Noetic</td>
  </tr>
</table>

## Overview
The **ROS Gesture Recognizer** is a vision-based gesture recognition system designed to interact with robotic platforms. 

It utilizes **[OpenCV](https://opencv.org/)**, **[MediaPipe](https://ai.google.dev/edge/mediapipe/solutions/guide)**, and **[TensorFlow](https://www.tensorflow.org/?hl=es)** to detect hand gestures and map them to predefined robotic actions such as movement commands.

The system is built on **[ROS Noetic](https://wiki.ros.org/noetic)** and runs on a **Raspberry Pi 4** equipped with a **Camera V2.1**.

This project is a Proof-Of-Concept to learn how to use ROS and take advantage of its features.

## Features
- **Hand Gesture Recognition**: Detects and classifies hand gestures using [MediaPipe](https://github.com/google-ai-edge/mediapipe).
- **Real-Time Processing**: Optimized for real-time gesture recognition on Raspberry Pi.
- **ROS Integration**: Publishes recognized gestures as **ROS topics** for use in robotic applications.
- **Custom Action Commands**: Converts gestures into robot commands like **STOP, GO LEFT, GO RIGHT, GO FORWARD**.
- **Performance Monitoring**: Includes a **metrics dashboard** to visualize processing delay and network latency.
- **Camera Integration**: Supports **Raspberry Pi Camera V2.1** for real-time image acquisition.
- **Pan-Tilt Mechanism**: Can be extended to control a **PCA9685 servo driver** for camera positioning.

### Prerequisites
- **Hardware**: Raspberry Pi 4, Camera V2.1, PCA9685 servo driver, pan-tilt camera mount.
- **OS**: Raspberry Pi OS 32-bit (Debian Buster recommended).
- **ROS**: Noetic (installed on Raspberry Pi and/or host machine).


## System Components
### 1. Gesture Recognition Module
- Captures images from the camera.
- Uses **MediaPipe Hands** to extract hand landmarks.
- Maps hand gestures to corresponding movement commands.
- Publishes commands to **ROS topics** (`/actions`).
- Publishes performance `Metric` data to **ROS topics** (`/metrics`).

### 2. Image Processing Pipeline
- Captures frames using **Picamera[array]**.
- Converts images to **OpenCV format** for processing.
- Overlays detected landmarks for visualization.

### 3. ROS Node Integration
- Runs as a **ROS node** (`gesture_recognizer`).
- Listens for camera input on `/camera/image_compressed`.
- Publishes action commands as **custom ROS messages** (`ActionCommand`).
- Publishes real-time performance metrics (`Metric`).

### 4. Performance Metrics Dashboard
- Monitors **processing delay** and **network latency**.
- Uses **PyQtGraph** for real-time visualization.
- Provides insights into system performance.

![architecture_whiteboard](https://github.com/user-attachments/assets/adfc2a45-0de6-4418-9665-3bfdcf887654)

## Installation
### 1. Clone the Repository
```bash
git clone https://github.com/your-repo/ros-gesture-recognizer.git
cd ros-gesture-recognizer
```

### 2. Install Dependencies
#### Python Dependencies
```bash
pip3 install opencv-python mediapipe tensorflow
```

### Raspberry Pi 4 - Setup

#### 1. Setup OS - Raspbian Buster 32 Bits
[Download ISO](https://downloads.raspberrypi.org/raspios_armhf/images/raspios_armhf-2021-05-28/2021-05-07-raspios-buster-armhf.zip)


#### 2. Install Python Dependencies
```bash
sudo apt install -y python3-pip python3-virtualenv  
pip3 install "picamera[array]"
pip3 install mediapipe --break-system-packages
pip3 install pyqtgraph PyQt5 numpy
```

#### 3. Install OpenCV 4.11 - 32 Bits Supported
- [Script to Install + Compile OpenCV 4.11](https://github.com/Qengineering/Install-OpenCV-Raspberry-Pi-32-bits/blob/main/OpenCV-4-11-0.sh/)

- Fix Building - Disable TBB when running `cmake`.

```bash
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
      -D ENABLE_NEON=ON \
      -D WITH_OPENMP=ON \
      -D WITH_OPENCL=OFF \
      -D BUILD_TIFF=ON \
      -D WITH_FFMPEG=ON \
      -D WITH_TBB=OFF \
      -D BUILD_TBB=OFF \
      -D WITH_GSTREAMER=ON \
      -D BUILD_TESTS=OFF \
      -D WITH_EIGEN=OFF \
      -D WITH_V4L=ON \
      -D WITH_LIBV4L=ON \
      -D WITH_VTK=OFF \
      -D WITH_QT=ON \
      -D WITH_PROTOBUF=OFF \
      -D OPENCV_ENABLE_NONFREE=ON \
      -D INSTALL_C_EXAMPLES=OFF \
      -D INSTALL_PYTHON_EXAMPLES=OFF \
      -D OPENCV_FORCE_LIBATOMIC_COMPILER_CHECK=1 \
      -D PYTHON3_PACKAGES_PATH=/usr/lib/python3/dist-packages \
      -D OPENCV_GENERATE_PKGCONFIG=ON \
      -D BUILD_EXAMPLES=OFF ..

make -j4
sudo make install
sudo apt update
```

#### 4. Install ROS Noetic in Debian Buster 32 bits
[Install ROS Noetic Steps](https://varhowto.com/install-ros-noetic-raspberry-pi-4/)

-- Modification to the step 6
```bash
# Add sensor_msgs and cv_bridge dependencies besides core ones
rosinstall_generator ros_comm sensor_msgs cv_bridge image_transport --rosdistro noetic --deps --wet-only --tar > noetic-ros_comm-wet.rosinstall
```

#### 5. Install PanTilt Driver (PCA9685)

[Guide Link](https://blog.garybricks.com/control-16-servos-with-raspberry-pi-pca9685-driver)

Install the driver:
```bash
git clone https://github.com/adafruit/Adafruit_Python_PCA9685.git
python3 setup.py install

python3 examples/simpletest.py
```

## Running the Project
### 1. On Desktop (ROS Environment)
```bash
roslaunch gesture_recognizer robot_run.launch
```

### 2. On Raspberry Pi
```bash
roslaunch raspi_image raspi_run.launch
```
