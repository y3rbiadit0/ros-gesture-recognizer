#!/usr/bin/python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import mediapipe as mp
from std_msgs.msg import String
from gesture_recognizer.msg import ActionCommand, Metric


# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)

class GestureRecognizer:
    """
    Determines which gesture is being made based on hand landmarks.
    """
    # Landmark index mapping
    WRIST = 0
    THUMB_TIP = 4
    INDEX_TIP = 8
    MIDDLE_TIP = 12
    RING_TIP = 16
    PINKY_TIP = 20

    THUMB_IP = 3
    INDEX_MCP = 5

    def __init__(self):
        pass

    def recognize(self, landmarks):

        # Get positions
        thumb_tip = landmarks.landmark[GestureRecognizer.THUMB_TIP]
        thumb_ip = landmarks.landmark[GestureRecognizer.THUMB_IP]
        index_tip = landmarks.landmark[GestureRecognizer.INDEX_TIP]
        middle_tip = landmarks.landmark[GestureRecognizer.MIDDLE_TIP]
        ring_tip = landmarks.landmark[GestureRecognizer.RING_TIP]
        pinky_tip = landmarks.landmark[GestureRecognizer.PINKY_TIP]

        wrist = landmarks.landmark[GestureRecognizer.WRIST]
        index_mcp = landmarks.landmark[GestureRecognizer.INDEX_MCP]

        # Check if all fingers are bent (FIST = STOP)
        if (
            index_tip.y > index_mcp.y
            and middle_tip.y > index_mcp.y
            and ring_tip.y > index_mcp.y
            and pinky_tip.y > index_mcp.y
        ):
            return self._createAction("STOP")

        # Check if thumb is extended to the left (GO LEFT)
        if (
            thumb_tip.x < thumb_ip.x  # Thumb is pointing left
            and index_tip.y > index_mcp.y  # Other fingers curled
            and wrist.z < 0 # Palm facing the camera
        ):
            return self._createAction("GO LEFT")

        # Check if thumb is extended to the right (GO RIGHT)
        if (
            thumb_tip.x > thumb_ip.x  # Thumb is pointing right
            and index_tip.y > index_mcp.y  # Other fingers curled
            and wrist.z > 0 # Palm facing away from camera
        ):
            return self._createAction("GO RIGHT")

        # Check if index finger is pointing up (GO FORWARD)
        if (
            index_tip.y < wrist.y  # Index finger above wrist
            and middle_tip.y > index_mcp.y  # Other fingers curled      
        ):
            return self._createAction("GO FORWARD")

        return None  # No recognized gesture
    
    def _createAction(self, action_str: str):
        action = ActionCommand()
        action.x_velocity = 1.0 # TODO - Set from params
        action.angular_velocity = 1.0 # TODO - Set from params
        action.action_type = action_str
        action.confidence = 0.0 # TODO

class GesturePublisher:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('image_subscriber', anonymous=True)
        self.bridge = CvBridge()
        # Hand Gesture - from landmarks
        self.gesture_recognizer = GestureRecognizer()
        self.action_publisher = rospy.Publisher('/actions', ActionCommand, queue_size=1)
        self.sub = rospy.Subscriber('/camera/image_compressed', 
                                  CompressedImage, 
                                  self._image_callback, 
                                  queue_size=1)
        
        self.metric_pub = rospy.Publisher('/metrics', Metric, queue_size=1)
        
        rospy.loginfo("Waiting for compressed images...")

    
    
    def _image_callback(self, msg):
        # Metric Message
        metric_msg = Metric()
        metric_msg.processing_delay = rospy.Time.now().to_sec()
        
        # Decode the compressed JPEG image
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # Get Network Latency
        metric_msg.network_latency = self._network_latency(msg)
        
        if cv_image is not None:
            
            # Process image
            frame_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            result = hands.process(frame_rgb)
            
            # Display the annotated image
            cv2.imshow("Received Frame with Hands", cv_image)
            # Draw hand landmarks on the original BGR image
            if result.multi_hand_landmarks:
                for hand_landmarks in result.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(cv_image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                    action = self.gesture_recognizer.recognize(hand_landmarks)
                    if action:
                        self.action_publisher.publish(action)
            
            self._publish_metrics(metric_msg)
            
            # Display the annotated image
            cv2.imshow("Received Frame with Hands", cv_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("User exited.")
        else:
            rospy.logwarn("Failed to decode image")
    
    def _network_latency(self, msg) -> float:
        """Get Network Latency coming from Raspberry Pi"""
        receive_time = rospy.Time.now()
        return (receive_time - msg.header.stamp).to_sec()
    
    def _publish_metrics(self, metric_msg: Metric):
        metric_msg.processing_delay = rospy.Time.now().to_sec() - metric_msg.processing_delay
        self.metric_pub.publish(metric_msg)  

    def shutdown(self):
        hands.close()  # Clean up MediaPipe resources
        cv2.destroyAllWindows()

def main():
    try:
        subscriber = GesturePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        subscriber.shutdown()

if __name__ == '__main__':
    main()

