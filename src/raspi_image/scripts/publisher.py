#!/usr/bin/python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

def publish_video():
    rospy.init_node('pi_camera_publisher', anonymous=True)
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))

    time.sleep(0.1)  # Camera warm-up

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        ros_image = bridge.cv2_to_imgmsg(image, "bgr8")

        image_pub.publish(ros_image)
        rospy.loginfo("Image Published")

        rawCapture.truncate(0)  # Clear buffer

        if rospy.is_shutdown():
            break

if __name__ == '__main__':
    try:
        publish_video()
    except rospy.ROSInterruptException:
        pass