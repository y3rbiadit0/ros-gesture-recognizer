#!/usr/bin/python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import mediapipe as mp

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)


class ImageSubscriber:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('image_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('/camera/image_compressed', 
                                  CompressedImage, 
                                  self._image_callback, 
                                  queue_size=1)
        
        rospy.loginfo("Waiting for compressed images...")

    def _image_callback(self, msg):
        # Decode the compressed JPEG image
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if cv_image is not None:
            # Convert BGR to RGB for MediaPipe
            frame_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Process the frame with MediaPipe Hands
            result = hands.process(frame_rgb)
            
            # Draw hand landmarks on the original BGR image
            if result.multi_hand_landmarks:
                for hand_landmarks in result.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(
                        cv_image,              # Draw on original BGR image
                        hand_landmarks,
                        mp_hands.HAND_CONNECTIONS
                    )
            
            # Display the annotated image
            cv2.imshow("Received Frame with Hands", cv_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("User exited.")
        else:
            rospy.logwarn("Failed to decode image")

    def shutdown(self):
        hands.close()  # Clean up MediaPipe resources
        cv2.destroyAllWindows()

def main():
    try:
        subscriber = ImageSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        subscriber.shutdown()

if __name__ == '__main__':
    main()