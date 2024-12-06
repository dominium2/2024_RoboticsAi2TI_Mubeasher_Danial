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
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.SYSTEM_DEFAULT))
        self.timer_period = 0.25  # Timer period in seconds
        self.blocked_since = None  # Timestamp when the robot got blocked
        self.laser_forward = float('inf')
        self.laser_frontLeft = float('inf')
        self.laser_frontRight = float('inf')
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def laser_callback(self,msg): 
        # Save the frontal laser scan info at 0° 
        self.laser_forward = msg.ranges[0] 
        degrees = 40

        self.laser_frontLeft = min(msg.ranges[0:degrees]) 
        self.laser_frontRight = min(msg.ranges[-degrees:]) 
        
    def motion(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0

        limitRange = 0.3

        self.get_logger().info('Left: "%s"' % (str(self.laser_frontLeft)))
        self.get_logger().info('Right: "%s"' % (str(self.laser_frontRight)))

    
        if self.laser_forward > limitRange and self.laser_frontLeft > limitRange and self.laser_frontRight > limitRange:
            self.get_logger().info("Forward")
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = 0.0
        else :    
            if (self.laser_frontLeft < self.laser_frontRight) :
                self.get_logger().info("Turn Right")
                self.cmd.angular.z = random.uniform(-0.5, 0.5)
                self.cmd.linear.x = 0.0
                time.sleep = random.uniform(0, 0.5)
            else :
                self.get_logger().info("Turn Left")
                self.cmd.angular.z = 0.8
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
