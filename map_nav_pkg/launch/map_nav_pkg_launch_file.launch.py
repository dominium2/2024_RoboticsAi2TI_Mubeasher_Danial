from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_cartographer',
            executable='cartographer',
            name='cartographer',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='map_nav_pkg',
            executable='map_nav',
            name='map_nav',
            output='screen'
        ),
        Node(
            package='map_nav_pkg',
            executable='additional_script',
            name='additional_script',
            output='screen'
        ),
    ])