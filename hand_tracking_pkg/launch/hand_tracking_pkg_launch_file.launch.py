from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hand_tracking_pkg',
            executable='hand_tracking',
            output='screen'),
    ])