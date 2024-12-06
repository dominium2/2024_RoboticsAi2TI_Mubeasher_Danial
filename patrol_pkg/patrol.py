import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import time
import random

class Patrol(Node):

    def __init__(self):
        super().__init__('patrol')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.timer_period = 1  # Timer period in seconds
        self.blocked_since = None  # Timestamp when the robot got blocked
        self.laser_forward = float('inf')
        self.laser_frontLeft = float('inf')
        self.laser_frontRight = float('inf')
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def laser_callback(self, msg):
        self.laser_forward = msg.ranges[359]
        self.laser_frontLeft = min(msg.ranges[0:20])
        self.laser_frontRight = min(msg.ranges[340:359])

    def motion(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0


        if self.laser_forward > 0.5 and self.laser_frontLeft > 0.5 and self.laser_frontRight > 0.5:
            self.cmd.linear.x = 0.2
            self.blocked_since = None  # Reset the blocked timestamp
        else:
            if self.blocked_since is None:
                self.blocked_since = time.time()

            elapsed_time = time.time() - self.blocked_since

            if elapsed_time >= 5:
                # Turn 180 degrees (~3.14159 radians)
                self.cmd.angular.z = 3.14159
                self.blocked_since = None  # Reset the blocked timestamp
            else:
                if self.laser_frontLeft < 0.5 and self.laser_frontRight < 0.5:
                    self.cmd.angular.z = 0.5 if self.laser_frontLeft > self.laser_frontRight else -0.5
                elif self.laser_frontLeft < 0.5:
                    self.cmd.angular.z = random.uniform(0, -2)
                    self.cmd.linear.x = 0.0
                elif self.laser_frontRight < 0.5:
                    self.cmd.angular.z = random.uniform(0, 2)
                    self.cmd.linear.x = 0.0

        self.publisher_.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    patrol = Patrol()
    rclpy.spin(patrol)
    patrol.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
