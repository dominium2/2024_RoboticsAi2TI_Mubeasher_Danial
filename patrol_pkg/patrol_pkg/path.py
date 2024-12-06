import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import time
import random




class Path(Node):

    def __init__(self):
        super().__init__('path')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.SYSTEM_DEFAULT))
        self.timer_period = 1  # Timer period in seconds
        self.blocked_since = None  # Timestamp when the robot got blocked
        self.laser_forward = float('inf')
        self.laser_frontLeft = float('inf')
        self.laser_frontRight = float('inf')
        self.cmd = Twist()

        # Tune these 2 parameters so robot turns exactly correct degrees and drives the exact distance.
        self.xSpeed = 0.117
        self.zSpeed = 3.3

        # self.timer = self.create_timer(self.timer_period, self.motion)
        self.motion()


    def laser_callback(self, msg):
        self.laser_forward = msg.ranges[0]
        self.laser_frontLeft = min(msg.ranges[0:20])
        self.laser_frontRight = min(msg.ranges[-20:])

    def move(self, linear_velocity, angular_velocity, duration):

        self.cmd.linear.x = linear_velocity
        self.cmd.angular.z = angular_velocity

        start_time = time.time()

        while time.time() - start_time < duration:
            self.publisher_.publish(self.cmd)
            time.sleep(0.01)  # sleep for 10ms

        self.stop_turtlebot()




    def stop_turtlebot(self):
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0  # No linear velocity
            stop_cmd.angular.z = 0.0  # No angular velocity

            # Publish the stop command
            self.publisher_.publish(stop_cmd)
            self.get_logger().info('Stop robot')
            time.sleep(0.1)  # sleep for 10ms



    def driveForward(self, distance):
        distance = distance*10
        self.get_logger().info('Drive forward: "%s" meter' % (str(distance/10)))
        self.move(self.xSpeed, 0.0,distance)

    def driveBackward(self, distance):
        distance = distance*10
        self.get_logger().info('Drive forward: "%s" meter' % (str(distance/10)))
        self.move(-1*self.xSpeed, 0.0,distance)


    def turnLeft(self, degrees):
        calcDegrees = (degrees / 90) * self.zSpeed
        self.get_logger().info('Turn left: "%s" degrees' % (str(degrees)))
        self.move(0.0, 0.5,calcDegrees)

    def turnRight(self, degrees):
        calcDegrees = (degrees / 90) * self.zSpeed
        self.get_logger().info('Turn right: "%s" degrees' % (str(degrees)))
        self.move(0.0, -0.5,calcDegrees)




    def motion(self):
    

        self.driveForward(0.5)
        self.turnLeft(90)
        self.driveForward(0.2)
        self.turnLeft(90)
        self.driveForward(0.5)
        self.turnLeft(90)
        self.driveForward(0.2)
        self.turnLeft(90)



def main(args=None):
    rclpy.init(args=args)
    path = Path()
    #rclpy.spin(path)
    path.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
