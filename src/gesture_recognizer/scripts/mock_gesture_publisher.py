#!/usr/bin/env python 3
import rospy
from std_msgs.msg import String

def mock_gesture_publisher():
    rospy.init_node('mock_gesture_publisher', anonymous= True)
    pub = rospy.Publisher('hand_gestures', String, queue_size=10)
    rate= rospy.Rate(1)

    gestures=['go forward', 'stop', 'go up', 'go forward' ,'stop', 'go down', 'stop', 'turn right', 'stop','go forward', 'turn left', 'go forward', 'stop']
    count=0
    
    while not rospy.is_shutdown():
        msg= String()
        msg.data= gestures[count % len(gestures)]

        rospy.loginfo(msg.data)
        pub.publish(msg)

        count += 1
        rate.sleep()

if __name__=="__main__":
    try:
        mock_gesture_publisher()
    except rospy.ROSInterruptException:
        pass