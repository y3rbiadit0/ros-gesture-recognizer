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

        self.step_size= 20 #for incremental movement 

        #Track position of camera
        self.current_horizontal = self.servo_center
        self.current_vertical = self.servo_center
        
        # ROS setup
        rospy.init_node('camera_servo_node', anonymous=True)
        self.sub = rospy.Subscriber('/camera_action', Int8, self.callback)
        
        # Set initial position
        self.set_servo_position_vertical(self.servo_center)
        self.set_servo_position_horizontal(self.servo_center)

        rospy.on_shutdown(self.shutdown)
        
    def set_servo_position_horizontal(self, pulse):
        horizontal_channel = 1
        # Ensure pulse stays within bounds
        pulse = max(self.servo_min, min(self.servo_max, pulse))
        self.pwm.set_pwm(horizontal_channel, 0, pulse)
        self.current_horizontal = pulse

    def set_servo_position_vertical(self, pulse):
        vertical_channel = 0
        # Ensure pulse stays within bounds
        pulse = max(self.servo_min, min(self.servo_max, pulse))
        self.pwm.set_pwm(vertical_channel, 0, pulse)
        self.current_vertical = pulse
        
    def callback(self, msg):
        command = msg.data
        if command == 1:  # Left
            new_pos = self.current_horizontal - self.step_size
            self.set_servo_position_horizontal(new_pos)
            rospy.loginfo(f"Moving camera left to {new_pos}")
        elif command == 2:  # Right
            new_pos = self.current_horizontal + self.step_size
            self.set_servo_position_horizontal(new_pos)
            rospy.loginfo(f"Moving camera right to {new_pos}")
        elif command == 3:  # Center
            self.set_servo_position_horizontal(self.servo_center)
            self.set_servo_position_vertical(self.servo_center)
            rospy.loginfo(f"Moving camera to center")
        elif command == 4: #Up
            new_pos = self.current_vertical + self.step_size
            self.set_servo_position_vertical(new_pos)
            rospy.loginfo(f"Moving camera up to {new_pos}")
        elif command == 5:
            new_pos = self.current_vertical - self.step_size
            self.set_servo_position_vertical(new_pos)
            rospy.loginfo(f"Moving camera down to {new_pos}")
        else:
            rospy.logwarn(f"Invalid command received: {command}")
            
    def shutdown(self):
        self.set_servo_position_vertical(self.servo_center)  # Center on shutdown
        self.set_servo_position_horizontal(self.servo_center)  # Center on shutdown
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = CameraServoNode()
        node.run()
    except rospy.ROSInterruptException:
        pass