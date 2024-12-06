import cv2
import os
import mediapipe as mp
import math
import time
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

# Setup MediaPipe
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.5, min_tracking_confidence=0.5)

# Camera instellingen
windowName = "turtleCam"
cv2.namedWindow(windowName)
vc = cv2.VideoCapture("http://192.168.0.56:8080/?action=stream")

def rescale_frame(frame, percent=75):
    width = int(frame.shape[1] * percent / 100)
    height = int(frame.shape[0] * percent / 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)

def calculate_angle(y, x):
    """Bereken de hoek van de wijsvinger in graden."""
    angle = math.degrees(math.atan2(-y, x))  # Negate y omdat het beeld coördinatensysteem inverted is
    if angle < -180:
        angle += 360
    elif angle > 180:
        angle -= 360
    return angle

class Patrol(Node):

    def __init__(self):
        super().__init__('patrol')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.SYSTEM_DEFAULT))
        self.timer_period = 0.25  # Timer period in seconds
        self.blocked_since = None  # Timestamp when the robot got blocked
        self.laser_forward = float('inf')
        self.laser_frontLeft = float('inf')
        self.laser_frontRight = float('inf')
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)
        self.finger_angle = 0.0  # Hoek van de wijsvinger

    def laser_callback(self, msg): 
        # Save the frontal laser scan info at 0° 
        self.laser_forward = msg.ranges[0] 
        degrees = 40
        self.laser_frontLeft = min(msg.ranges[0:degrees]) 
        self.laser_frontRight = min(msg.ranges[-degrees:]) 
        
    def update_finger_angle(self):
        global vc

        rval, frame = vc.read()
        if rval:
            frame = rescale_frame(frame, 100)
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Process the frame met MediaPipe
            result = hands.process(rgb_frame)

            if result.multi_hand_landmarks:
                for hand_landmarks in result.multi_hand_landmarks:
                    # Wijsvingertop en -basis coördinaten ophalen
                    index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
                    index_base = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]

                    # Pixel coördinaten berekenen
                    height, width, _ = frame.shape
                    tip_x, tip_y = int(index_tip.x * width), int(index_tip.y * height)
                    base_x, base_y = int(index_base.x * width), int(index_base.y * height)

                    # Verschillen in coördinaten berekenen
                    dx = tip_x - base_x
                    dy = tip_y - base_y

                    # Hoek berekenen
                    self.finger_angle = calculate_angle(dy, dx) - 90

                    # Visualisatie op frame
                    cv2.circle(frame, (tip_x, tip_y), 10, (0, 255, 0), -1)
                    cv2.circle(frame, (base_x, base_y), 10, (255, 0, 0), -1)
                    cv2.line(frame, (base_x, base_y), (tip_x, tip_y), (255, 255, 255), 2)

            cv2.imshow(windowName, frame)
            cv2.waitKey(1)

    def motion(self):
        # Update de hoek van de vinger voordat we de beweging berekenen
        self.update_finger_angle()

        # Stel de standaardwaarden in
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0

        limitRange = 0.3

        # Logging van sensorwaarden
        self.get_logger().info('Left: "%s"' % (str(self.laser_frontLeft)))
        self.get_logger().info('Right: "%s"' % (str(self.laser_frontRight)))

        # Robotbeweging op basis van lasersensoren en vingerhoek
        if self.laser_forward > limitRange and self.laser_frontLeft > limitRange and self.laser_frontRight > limitRange:
            self.get_logger().info("Forward")
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = self.finger_angle / 180.0  # De hoek van de vinger bepaalt de draaisnelheid (geschaald)
        else:
            if (self.laser_frontLeft < self.laser_frontRight):
                self.get_logger().info("Turn Right")
                self.cmd.angular.z = random.uniform(-0.5, 0.5)
                self.cmd.linear.x = 0.0
                time.sleep(random.uniform(0, 0.5))
            else:
                self.get_logger().info("Turn Left")
                self.cmd.angular.z = 0.8
                self.cmd.linear.x = 0.0

        # Publiceer de aangepaste cmd_vel
        self.publisher_.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    patrol = Patrol()
    try:
        rclpy.spin(patrol)
    except KeyboardInterrupt:
        pass
    patrol.destroy_node()
    rclpy.shutdown()
    vc.release()
    cv2.destroyAllWindows()
    hands.close()

if __name__ == '__main__':
    main()
