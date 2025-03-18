#!/usr/bin/python3

import rospy
from std_msgs.msg import Int8
from gesture_recognizer.srv import MoveCamera, MoveCameraResponse
import sys
import tty
import termios

class CameraControlService:
    def __init__(self):
        rospy.init_node('camera_control_service', anonymous=True)
        
        # Publisher to send commands to the Raspberry Pi
        self.pub = rospy.Publisher('/camera_action', Int8, queue_size=1)
        
        # Service to accept move commands
        self.srv = rospy.Service('/move_camera', MoveCamera, self.handle_move_camera)
        
        self.running = True
        rospy.on_shutdown(self.shutdown)
        
    def handle_move_camera(self, req):
        command = req.command
        print(command)
        if command in [1, 2, 3, 4, 5]:  # Valid commands: 1=left, 2=right, 3=center, 4=up, 5=down
            self.pub.publish(command)
            return MoveCameraResponse(True, f"Sent command {command} to camera")
        return MoveCameraResponse(False, "Invalid command (use 1, 2, 3, 4 or 5)")
        
    def getch(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
        
    def keyboard_control(self):
        print("Camera control - Use keys:")
        print("'a': Left, 'd': Right, 's': Center, 'w' : Up, 'e': Down, 'q': Quit")
        print("Service available at '/move_camera'")
        
        while self.running and not rospy.is_shutdown():
            char = self.getch()
            
            if char == 'a':  # Left
                self.pub.publish(1)
                print("Sent: Move left")
            elif char == 'd':  # Right
                self.pub.publish(2)
                print("Sent: Move right")
            elif char == 's':  # Center
                self.pub.publish(3)
                print("Sent: Move to center")
            elif char == 'w':  # Center
                self.pub.publish(4)
                print("Sent: Move to up")
            elif char == 'e':  # Center
                self.pub.publish(5)
                print("Sent: Move to down")
            elif char == 'q':  # Quit
                self.running = False
                print("Shutting down")
                
    def shutdown(self):
        self.running = False
        self.pub.publish(3)  # Center camera on shutdown
        
    def run(self):
        try:
            self.keyboard_control()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    try:
        controller = CameraControlService()
        controller.run()
        rospy.spin()
    except Exception as e:
        print(f"Error: {e}")