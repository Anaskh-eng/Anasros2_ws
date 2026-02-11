from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pub_sub_ex',
            executable='pub',
            name='publisher',
            arguments=['--ros-args', '--log-level', 'WARN']  # Set log level to WARN
        ),
        Node(
            package='pub_sub_ex',
            executable='sub',
            name='subscriber',
        ),
    ])