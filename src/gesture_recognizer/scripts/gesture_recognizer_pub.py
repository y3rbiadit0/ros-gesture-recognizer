#!/usr/bin/python3
import rospy
#import finger_recognition as fr
from std_msgs.msg import String

current_gesture="stop"
def gesture_callback(msg):
    global current_gesture

    current_gesture= msg #fr.detect_gesture(handlandmark)
    rospy.loginfo(current_gesture)

def gesture_publisher():
    rospy.init_node('gesture_publisher', anonymous= True)
    pub = rospy.Publisher('actions', String, queue_size=10)
    rate= rospy.Rate(10)

    rospy.Subscriber('/hand_gestures', String, gesture_callback) #Instead of string, should be handlandmark

    while not rospy.is_shutdown():
        rospy.loginfo(current_gesture)
        pub.publish(current_gesture)
        rate.sleep()
    

if __name__ == '__main__':
    try:
        gesture_publisher()
    except rospy.ROSInterruptException:
        pass
