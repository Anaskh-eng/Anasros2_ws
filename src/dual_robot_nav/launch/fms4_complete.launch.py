#!/usr/bin/env python3
"""
Complete working FMS launch file
Replicates the manual spawning method that works
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Get package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_dual_robot = get_package_share_directory('dual_robot_nav')
    
    # World file
    world_file = os.path.join(pkg_dual_robot, 'worlds', 'fms_layout4.world')
    
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 1. Launch Gazebo with ROS2 plugins and custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    
    # 2. Robot State Publisher (publishes robot_description)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    # 3. Spawn TurtleBot3 (delayed to ensure Gazebo is ready)
    spawn_turtlebot = TimerAction(
        period=3.0,  # Wait 3 seconds for Gazebo to fully start
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                    '-database', 'turtlebot3_waffle_pi', 
                    '-entity', 'turtlebot3',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.01'
                ],
                output='screen'
            )
        ]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add all actions in order
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_turtlebot)
    
    return ld
