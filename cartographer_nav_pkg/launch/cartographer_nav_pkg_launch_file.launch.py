from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_cartographer',
            executable='cartographer_node',
            name='cartographer',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='turtlebot3_patrol',
            executable='cartographer_nav',
            name='cartographer_nav',
            output='screen'
        ),
        Node(
            package='turtlebot3_teleop',
            executable='teleop_keyboard',
            name='teleop_keyboard',
            output='screen'
        )
    ])