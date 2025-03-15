#!/usr/bin/python3
import cv2
import mediapipe as mp

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)

# Open Webcam
cap = cv2.VideoCapture(2)

def detect_gesture(landmarks):
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

    # Get positions
    thumb_tip = landmarks.landmark[THUMB_TIP]
    thumb_ip = landmarks.landmark[THUMB_IP]
    index_tip = landmarks.landmark[INDEX_TIP]
    middle_tip = landmarks.landmark[MIDDLE_TIP]
    ring_tip = landmarks.landmark[RING_TIP]
    pinky_tip = landmarks.landmark[PINKY_TIP]

    wrist = landmarks.landmark[WRIST]
    index_mcp = landmarks.landmark[INDEX_MCP]

    # Check if all fingers are bent (FIST = STOP)
    if (
        index_tip.y > index_mcp.y
        and middle_tip.y > index_mcp.y
        and ring_tip.y > index_mcp.y
        and pinky_tip.y > index_mcp.y
    ):
        return "STOP"

    # Check if thumb is extended to the left (GO LEFT)
    if (
        thumb_tip.x < thumb_ip.x  # Thumb is pointing left
        and index_tip.y > index_mcp.y  # Other fingers curled
        and wrist.z < 0 # Palm facing the camera
    ):
        return "GO LEFT"

    # Check if thumb is extended to the right (GO RIGHT)
    if (
        thumb_tip.x > thumb_ip.x  # Thumb is pointing right
        and index_tip.y > index_mcp.y  # Other fingers curled
        and wrist.z > 0 # Palm facing away from camera
    ):
        return "GO RIGHT"

    # Check if index finger is pointing up (GO FORWARD)
    if (
        index_tip.y < wrist.y  # Index finger above wrist
        and middle_tip.y > index_mcp.y  # Other fingers curled      
    ):
        return "GO FORWARD"

    return None  # No recognized gesture

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = hands.process(frame_rgb)

    if result.multi_hand_landmarks:
        for hand_landmarks in result.multi_hand_landmarks:
            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            # Detect gesture
            command = detect_gesture(hand_landmarks)
            if command:
                cv2.putText(frame, command, (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow("Hand Gesture Control", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
