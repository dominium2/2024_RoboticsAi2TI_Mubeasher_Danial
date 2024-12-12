import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class HandTrackingNode(Node):
    def __init__(self):
        super().__init__("hand_tracking_node")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7, min_tracking_confidence=0.7)
        self.mp_draw = mp.solutions.drawing_utils
        self.cap = cv2.VideoCapture(0)
        self.closed_hand_threshold = 0.2
        self.middle_finger_threshold = 0.1

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame from webcam")
            return

        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)

        command = "STOP"  # Default to STOP if no hand is detected
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                # Check for middle finger gesture for spinning
                if self.is_middle_finger_raised(hand_landmarks):
                    command = "SPIN"
                    self.publish_command(spin=True)
                else:
                    # Check if the hand is open or closed
                    is_open = self.is_hand_open(hand_landmarks)
                    # If hand is closed, move forward; if open, stop
                    command = "START" if not is_open else "STOP"  # Reverse the displayed command
                    self.publish_command(is_closed=not is_open)  # Moving forward when hand is closed

                # Print detected fingers for debugging
                detected_fingers = self.detect_fingers(hand_landmarks)
                print(f"Detected Fingers: {detected_fingers}")
        else:
            # If no hand is detected, stop the robot
            self.publish_command(is_closed=False)  # Send stop command
            command = "STOP"  # Update the command to STOP when no hand is detected

        cv2.putText(frame, command, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow("Hand Tracking", frame)


    def is_hand_open(self, hand_landmarks):
        """
        Checks if the hand is open (all five fingers raised).
        Args:
            hand_landmarks: The landmarks of the detected hand.
        Returns:
            bool: True if all five fingers are raised, False otherwise.
        """
        landmarks = hand_landmarks.landmark

        # Thumb: Tip (4) is further from the palm (MCP joint, 2) on the x-axis
        thumb_open = landmarks[self.mp_hands.HandLandmark.THUMB_TIP].x < landmarks[self.mp_hands.HandLandmark.THUMB_MCP].x

        # Fingers: Tip is higher (y-coordinate smaller) than the PIP joint for each finger
        index_open = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y < landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_PIP].y
        middle_open = landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y < landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP].y
        ring_open = landmarks[self.mp_hands.HandLandmark.RING_FINGER_TIP].y < landmarks[self.mp_hands.HandLandmark.RING_FINGER_PIP].y
        pinky_open = landmarks[self.mp_hands.HandLandmark.PINKY_TIP].y < landmarks[self.mp_hands.HandLandmark.PINKY_PIP].y

        # Check if all fingers and thumb are open
        return thumb_open and index_open and middle_open and ring_open and pinky_open


    def detect_fingers(self, hand_landmarks):
        """
        Detect which fingers are raised.
        Args:
            hand_landmarks: The landmarks of the detected hand.
        Returns:
            dict: Dictionary indicating whether each finger is raised.
        """
        landmarks = hand_landmarks.landmark

        fingers = {
            "Thumb": landmarks[self.mp_hands.HandLandmark.THUMB_TIP].x < landmarks[self.mp_hands.HandLandmark.THUMB_MCP].x,
            "Index": landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y < landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_PIP].y,
            "Middle": landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y < landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP].y,
            "Ring": landmarks[self.mp_hands.HandLandmark.RING_FINGER_TIP].y < landmarks[self.mp_hands.HandLandmark.RING_FINGER_PIP].y,
            "Pinky": landmarks[self.mp_hands.HandLandmark.PINKY_TIP].y < landmarks[self.mp_hands.HandLandmark.PINKY_PIP].y
        }
        return fingers

    def is_middle_finger_raised(self, hand_landmarks):
        """
        Checks if the middle finger is raised (middle finger gesture).
        Args:
            hand_landmarks: The landmarks of the detected hand.
        Returns:
            bool: True if the middle finger is raised, False otherwise.
        """
        middle_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
        index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        ring_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP]

        middle_height = middle_tip.y
        index_height = index_tip.y
        ring_height = ring_tip.y

        return middle_height < index_height - self.middle_finger_threshold and middle_height < ring_height - self.middle_finger_threshold

    def publish_command(self, is_closed=False, spin=False):
        twist = Twist()
        if spin:
            twist.angular.z = 1.0  # Spin at a fixed angular velocity
        elif is_closed:
            twist.linear.x = 0.2  # Move forward when hand is closed
        else:
            twist.linear.x = 0.0  # Stop when hand is open or no hand is detected
        self.publisher.publish(twist)
    

    def run(self):
        while rclpy.ok():
            self.process_frame()
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        self.cap.release()
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    hand_tracking_node = HandTrackingNode()
    try:
        hand_tracking_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        hand_tracking_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
