from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roller_control',
            executable='base_controller',
        ),
        Node(
            package='roller_control',
            executable='roller_controller',
        ),
        Node(
            package='roller_control',
            executable='roller_gui',
        ),
        Node(
            package='roller_control',
            executable='navigator',
        ),
    ])
