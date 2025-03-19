#!/usr/bin/env python3

import rospy
import sys
import tty
import termios
from gesture_recognizer.srv import MoveCamera, MoveCameraRequest

class KeyboardCameraClient:
    def __init__(self):
        rospy.init_node('keyboard_camera_client', anonymous=True)
        self.running = True
        rospy.on_shutdown(self.shutdown)
        
    def move_camera(self, command):
        rospy.wait_for_service('/move_camera')
        try:
            move_camera = rospy.ServiceProxy('/move_camera', MoveCamera)
            resp = move_camera(command)
            print(f"Command {command}: {resp.message} - Success: {resp.success}")
            return resp.success
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False

    def getch(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def run(self):
        print("Camera control - Use keys:")
        print("'a': Left, 'd': Right, 's': Center, 'w': Up, 'e': Down, 'q': Quit")
        print("Calling service '/move_camera'")

        while self.running and not rospy.is_shutdown():
            char = self.getch()
            
            if char == 'a':  # Left
                self.move_camera(1)
            elif char == 'd':  # Right
                self.move_camera(2)
            elif char == 's':  # Center
                self.move_camera(3)
            elif char == 'w':  # Up
                self.move_camera(4)
            elif char == 'e':  # Down
                self.move_camera(5)
            elif char == 'q':  # Quit
                self.running = False
                print("Shutting down")
                
    def shutdown(self):
        self.running = False
        print("Keyboard client stopped")

if __name__ == '__main__':
    try:
        client = KeyboardCameraClient()
        client.run()
    except rospy.ROSInterruptException:
        pass