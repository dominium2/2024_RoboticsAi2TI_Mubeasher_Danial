import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class TurtleBot3Driver(Node):
    def __init__(self):
        super().__init__('turtlebot3_driver')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.initial_position = None
        self.target_distance = 1.0  # meters

    def odom_callback(self, msg):
        if self.initial_position is None:
            self.initial_position = msg.pose.pose

        current_position = msg.pose.pose
        dx = current_position.position.x - self.initial_position.position.x
        dy = current_position.position.y - self.initial_position.position.y
        distance_moved = (dx ** 2 + dy ** 2) ** 0.5

        if distance_moved < self.target_distance:
            self.drive_forward()
        else:
            self.stop_moving()
            self.destroy_node()

    def drive_forward(self):
        msg = Twist()
        msg.linear.x = 0.1  # meters/sec
        self.publisher_.publish(msg)

    def stop_moving(self):
        msg = Twist()
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    turtlebot3_driver = TurtleBot3Driver()
    rclpy.spin(turtlebot3_driver)
    turtlebot3_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
