from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='auto-obst-av_pkg',
            executable='auto-obst-av',
            output='screen'),
    ])