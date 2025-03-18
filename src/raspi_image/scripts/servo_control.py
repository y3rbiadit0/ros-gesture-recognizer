#!/usr/bin/python3

import rospy
from std_msgs.msg import Int8
import Adafruit_PCA9685

class CameraServoNode:
    def __init__(self):
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)
        
        # Servo pulse lengths
        self.servo_min = 150    # Left position
        self.servo_center = 375 # Center position
        self.servo_max = 600    # Right position
        
        # ROS setup
        rospy.init_node('camera_servo_node', anonymous=True)
        self.sub = rospy.Subscriber('/camera_action', Int8, self.callback)
        
        # Set initial position
        self.set_servo_position_vertical(self.servo_center)
        rospy.on_shutdown(self.shutdown)
        
    def set_servo_position_horizontal(self, pulse):
        horizontal_channel = 1
        self.pwm.set_pwm(horizontal_channel, 0, pulse)

    def set_servo_position_vertical(self, pulse):
        vertical_channel = 0
        self.pwm.set_pwm(vertical_channel, 0, pulse)
        
    def callback(self, msg):
        command = msg.data
        if command == 1:  # Left
            self.set_servo_position_horizontal(self.servo_min)
            rospy.loginfo("Moving camera left")
        elif command == 2:  # Right
            self.set_servo_position_horizontal(self.servo_max)
            rospy.loginfo("Moving camera right")
        elif command == 3:  # Center
            self.set_servo_position_horizontal(self.servo_center)
            rospy.loginfo("Moving camera to center")
        elif command == 4:
            self.set_servo_position_vertical(self.servo_max)
            rospy.loginfo("Moving camera to up")
        elif command == 5:
            self.set_servo_position_vertical(self.servo_min)
            rospy.loginfo("Moving camera to down")
        else:
            rospy.logwarn(f"Invalid command received: {command}")
            
    def shutdown(self):
        self.set_servo_position_vertical(self.servo_center)  # Center on shutdown
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = CameraServoNode()
        node.run()
    except rospy.ROSInterruptException:
        pass