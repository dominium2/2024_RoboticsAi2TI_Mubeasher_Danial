from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    patrol_node =Node(
            package='patrol_pkg',
            executable='patrol',
            output='screen')

    odom_node =Node(
        package='patrol_pkg',
        executable='odom',
        output='screen')


    ld.add_action(patrol_node)
    ld.add_action(odom_node)

    return ld