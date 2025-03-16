#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import threading
import cv2
import numpy as np

class PiCameraPublisher:
    def __init__(self):
        rospy.init_node('pi_camera_publisher', anonymous=True)
        self.image_pub = rospy.Publisher('/camera/image_compressed', 
                                       CompressedImage, 
                                       queue_size=1)
        self.bridge = CvBridge()
        
        # Optimize camera settings
        self.camera = PiCamera()
        self.camera.resolution = (320, 240)
        self.camera.framerate = 30
        self.camera.vflip = False
        
        # Pre-allocate buffer
        self.raw_capture = PiRGBArray(self.camera, size=(320, 240))
        self.running = True
        
        # Warm-up
        time.sleep(0.1)
        
        # Start publishing in separate thread
        self.publish_thread = threading.Thread(target=self._publish_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()

    def _publish_loop(self):
        for frame in self.camera.capture_continuous(self.raw_capture, 
                                                  format="bgr", 
                                                  use_video_port=True,
                                                  splitter_port=0):
            if not self.running or rospy.is_shutdown():
                break
                
            # Get image and compress it using OpenCV
            image = frame.array
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            # Encode to JPEG with quality parameter (0-100, lower = smaller size, faster)
            success, encoded_image = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            if success:
                msg.data = encoded_image.tobytes()
                self.image_pub.publish(msg)
            
            self.raw_capture.truncate(0)

    def shutdown(self):
        self.running = False
        self.publish_thread.join()
        self.camera.close()

def main():
    try:
        publisher = PiCameraPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        publisher.shutdown()

if __name__ == '__main__':
    main()