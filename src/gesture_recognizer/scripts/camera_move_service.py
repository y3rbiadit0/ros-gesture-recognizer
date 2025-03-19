#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8
from gesture_recognizer.srv import MoveCamera, MoveCameraResponse

class CameraControlService:
    def __init__(self):
        rospy.init_node('camera_control_service', anonymous=True)
        
        # Publisher to send commands to the Raspberry Pi
        self.pub = rospy.Publisher('/camera_action', Int8, queue_size=1)
        
        # Service to accept move commands
        self.srv = rospy.Service('/move_camera', MoveCamera, self.handle_move_camera)
        
        rospy.loginfo("Camera control service ready at '/move_camera'")
        rospy.on_shutdown(self.shutdown)
        
    def handle_move_camera(self, req):
        command = req.command
        if command in [1, 2, 3, 4, 5]:  # Valid commands: 1=left, 2=right, 3=center, 4=up, 5=down
            self.pub.publish(command)
            return MoveCameraResponse(True, f"Sent command {command} to camera")
        return MoveCameraResponse(False, "Invalid command (use 1, 2, 3, 4, or 5)")
        
    def shutdown(self):
        self.pub.publish(3)  # Center camera on shutdown
        rospy.loginfo("Service shutdown, camera centered")

if __name__ == '__main__':
    try:
        controller = CameraControlService()
        rospy.spin()  # Keep the service running
    except rospy.ROSInterruptException:
        pass