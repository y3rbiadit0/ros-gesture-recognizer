#!/usr/bin/python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def image_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    cv2.imshow("Received Frame", cv_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("User exited.")

def receive_video():
    rospy.init_node('image_subscriber', anonymous=True)
    rospy.Subscriber('/camera/image_raw', Image, image_callback)

    rospy.loginfo("Waiting for images...")
    rospy.spin()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        receive_video()
    except rospy.ROSInterruptException:
        pass
