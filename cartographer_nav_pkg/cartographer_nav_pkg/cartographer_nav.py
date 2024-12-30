# FILE: turtlebot3_patrol/turtlebot3_patrol/cartographer_nav.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class CartographerNav(Node):
    def __init__(self):
        super().__init__('cartographer_nav')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.twist = Twist()
        self.obstacle_distance = float('inf')
        self.patrol_state = 'start'
        self.patrol_steps = 0

    def odom_callback(self, msg):
        # Process odometry data
        pass

    def scan_callback(self, msg):
        # Process Lidar data
        self.obstacle_distance = min(msg.ranges)

    def timer_callback(self):
        # Control logic for patrolling
        if self.obstacle_distance < 0.5:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.5
        else:
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.0

        # Simple patrol logic
        if self.patrol_steps < 100:
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.0
        elif self.patrol_steps < 200:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.5
        else:
            self.patrol_steps = 0

        self.patrol_steps += 1
        self.publisher_.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = CartographerNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()