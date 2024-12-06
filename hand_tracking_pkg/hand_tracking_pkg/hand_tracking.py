import rclpy
from geometry_msgs.msg import Twist
import cv2
import mediapipe as mp
from cv_bridge import CvBridge
from rclpy.node import Node

class HandTrackingNode(Node):
    def __init__(self):
        super().__init__('hand_tracking_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cap = cv2.VideoCapture(0)  # Use the default webcam
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands()
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Convert frame to RGB
        results = self.hands.process(frame_rgb)  # Process the frame with MediaPipe

        twist_msg = Twist()

        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            # Use hand landmarks to determine hand state
            if self.is_hand_open(hand_landmarks):
                twist_msg.linear.x = 0.0  # Stop the robot
            else:
                twist_msg.linear.x = 0.1  # Move the robot forward

        # Publish the twist message to /cmd_vel
        self.publisher.publish(twist_msg)

        # Show the webcam feed with hand landmarks drawn
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)  # Convert back to BGR for OpenCV
        if results.multi_hand_landmarks:
            for landmarks in results.multi_hand_landmarks:
                mp.solutions.drawing_utils.draw_landmarks(frame_bgr, landmarks, self.mp_hands.HAND_CONNECTIONS)

        # Display the frame with landmarks
        cv2.imshow('Hand Tracking', frame_bgr)

        # Exit condition: press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()

    def is_hand_open(self, landmarks):
        # Example condition based on finger positions (implement a better one for your use case)
        thumb_tip = landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
        index_tip = landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        if index_tip.y < thumb_tip.y:  # Hand closed
            return False
        return True

def main(args=None):
    rclpy.init(args=args)
    node = HandTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
