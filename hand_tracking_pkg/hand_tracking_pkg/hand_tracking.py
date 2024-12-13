import cv2
import mediapipe as mp
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

class HandTrackingNode(Node):
    def __init__(self):
        super().__init__("hand_tracking_node")
        
        # Publisher for robot commands
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscriber for laser scan data (obstacle avoidance)
        self.obstacle_avoidance_subscriber = self.create_subscription(
            LaserScan, "/scan", self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Mediapipe hand tracking setup
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7, min_tracking_confidence=0.7)
        self.mp_draw = mp.solutions.drawing_utils
        
        # Variables for hand tracking and obstacle avoidance
        self.hand_command = Twist()  # Command for hand tracking (stop/start/spin)
        self.obstacle_command = Twist()  # Command for obstacle avoidance (stop only)
        
        # Laser forward distance for obstacle detection
        self.laser_forward = float('inf')  # Default to no obstacle detected

        # Webcam setup
        self.cap = cv2.VideoCapture(0)

        # Set camera resolution and FPS to improve performance
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Set width
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Set height
        self.cap.set(cv2.CAP_PROP_FPS, 30)  # Set frame rate to 30 FPS

        # Time tracking for hand tracking, obstacle avoidance, and laser scanning
        self.last_hand_tracking_time = time.time()
        self.last_obstacle_check_time = time.time()
        self.last_laser_scan_time = time.time()
        self.last_command_time = time.time()  # Time tracking for command publishing

    def laser_callback(self, msg):
        # Process laser scan for obstacle avoidance (only every 1 second)
        current_time = time.time()
        if current_time - self.last_laser_scan_time >= 1.0:  # Check every 1 second
            forward_index_start = 0
            forward_index_end = 20  # Adjust range for front laser
            front_ranges = msg.ranges[forward_index_start:forward_index_end]
            self.laser_forward = min(front_ranges)

            # Set obstacle avoidance command based on laser scan
            if self.laser_forward == float('inf'):
                self.obstacle_command.linear.x = 0.0  # No obstacle detected, do nothing
            elif self.laser_forward < 0.5:
                self.obstacle_command.linear.x = 0.0  # Stop if obstacle is too close
                self.obstacle_command.angular.z = 0.0  # No rotation for obstacle avoidance

            self.get_logger().info(f"Obstacle distance: {self.laser_forward:.2f} m")
            self.last_laser_scan_time = current_time  # Update last laser scan time

    def publish_command(self):
        # Prioritize obstacle avoidance: Stop if obstacle is detected
        if self.laser_forward < 0.5:
            self.get_logger().info("Obstacle detected, stopping robot.")
            self.publisher.publish(self.obstacle_command)  # Stop command
            return  # Skip sending any other command if obstacle is too close

        # If no hand is detected, stop the robot
        if self.hand_command.linear.x == 0.0 and self.hand_command.angular.z == 0.0:
            self.get_logger().info("No hand detected, stopping robot.")
            self.publisher.publish(self.obstacle_command)  # Stop the robot
        else:
            # Send hand tracking command if a gesture is detected
            self.publisher.publish(self.hand_command)  # Hand tracking command (START, STOP, SPIN)

    def process_frame(self):
        # Process frame for hand tracking gestures
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame from webcam")
            return

        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                command = self.detect_gesture(hand_landmarks)
                
                # If gesture detected, send corresponding command
                if command:
                    self.hand_command = command
        else:
            # No hand detected, stop the robot
            self.hand_command = Twist()  # STOP command

        # Show the video feed with hand tracking
        cv2.imshow("Hand Tracking", frame)
        cv2.waitKey(1)

    def detect_gesture(self, hand_landmarks):
        landmarks = hand_landmarks.landmark

        # Check if hand is open (STOP gesture)
        if self.is_hand_open(landmarks):
            return Twist()  # STOP command
        
        # Check if middle finger is raised (SPIN gesture)
        if self.is_middle_finger_raised(landmarks):
            twist = Twist()
            twist.angular.z = 1.0  # SPIN command
            return twist
        
        # If hand is closed (START gesture)
        if not self.is_hand_open(landmarks):
            twist = Twist()
            twist.linear.x = 0.1  # START command (move forward)
            return twist

        return None  # No gesture detected

    def is_hand_open(self, landmarks):
        # Check thumb and finger positions to determine if the hand is open
        thumb_open = landmarks[self.mp_hands.HandLandmark.THUMB_TIP].x < landmarks[self.mp_hands.HandLandmark.THUMB_MCP].x
        index_open = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y < landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_PIP].y
        middle_open = landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y < landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP].y
        ring_open = landmarks[self.mp_hands.HandLandmark.RING_FINGER_TIP].y < landmarks[self.mp_hands.HandLandmark.RING_FINGER_PIP].y
        pinky_open = landmarks[self.mp_hands.HandLandmark.PINKY_TIP].y < landmarks[self.mp_hands.HandLandmark.PINKY_PIP].y

        return thumb_open and index_open and middle_open and ring_open and pinky_open

    def is_middle_finger_raised(self, landmarks):
        middle_tip = landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
        index_tip = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        ring_tip = landmarks[self.mp_hands.HandLandmark.RING_FINGER_TIP]

        middle_height = middle_tip.y
        index_height = index_tip.y
        ring_height = ring_tip.y

        return middle_height < index_height - 0.1 and middle_height < ring_height - 0.1

    def run(self):
        while rclpy.ok():
            # Process the camera frame continuously at a high rate (e.g., 30 FPS)
            self.process_frame()

            # Time-based checks for hand tracking and obstacle avoidance
            current_time = time.time()

            # Process hand tracking every 0.5 seconds
            if current_time - self.last_hand_tracking_time >= 0.5:
                self.last_hand_tracking_time = current_time  # Update last hand tracking time
                self.publish_command()  # Send the hand tracking or obstacle avoidance command

            # Publish the command based on obstacle avoidance and hand tracking
            if current_time - self.last_command_time >= 0.5:
                self.last_command_time = current_time  # Update the last command publishing time

            rclpy.spin_once(self)  # Process any new messages

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
