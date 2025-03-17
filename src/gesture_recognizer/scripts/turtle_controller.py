#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class TurtleController:
    def __init__(self):
        rospy.init_node('turtle_controller', anonymous=True)
        self.velocity_publisher_turtle = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.velocity_publisher_gazebo = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/actions', String, self.action_callback)

        self.vel_msg= Twist()
        self.vel_msg.linear.x=0.0
        self.vel_msg.linear.y=0.0
        self.vel_msg.linear.z=0.0
        self.vel_msg.angular.x=0.0
        self.vel_msg.angular.y=0.0
        self.vel_msg.angular.z=0.0

        rospy.spin()

    def action_callback(self, msg):
        action = msg.data.lower()

        if action == "go forward" or action == "go up":
            self.vel_msg.linear.x= 1.0
        elif action == "stop":
            self.vel_msg.linear.x=0.0
        elif action =="go down":
            self.vel_msg.linear.x=-1.0
        elif action =="turn right":
            self.vel_msg.angular.z=-1
        elif action == "turn left":
            self.vel_msg.angular.z=1
        else:
            rospy.logwarn("Unknown action: %s", action)
            return
        self.velocity_publisher_turtle.publish(self.vel_msg)
        self.velocity_publisher_gazebo.publish(self.vel_msg)

if __name__=='__main__':
    try:
        TurtleController()
    except rospy.ROSInterruptException:
        pass