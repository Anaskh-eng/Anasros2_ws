# dual_robot_nav/launch/dual_robot_navigation.launch.py

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    dual_robot_nav_dir = get_package_share_directory('dual_robot_nav')
    turtlebot3_dir = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Robot 1 parameters
    robot1_x = '0.0'
    robot1_y = '0.0'
    robot1_namespace = 'robot1'
    
    # Robot 2 parameters
    robot2_x = '0.0'
    robot2_y = '1.0'
    robot2_namespace = 'robot2'
    
    # Launch world (your custom world)
    world_file = os.path.join(dual_robot_nav_dir, 'worlds', 'fms_layout2.world')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_file}.items()
    )
    
    # Robot 1 spawn
    robot1_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot1',
            '-file', os.path.join(turtlebot3_dir, 'models', 'turtlebot3_waffle_pi', 'model.sdf'),
            '-x', robot1_x,
            '-y', robot1_y,
            '-z', '0.01',
            '-robot_namespace', robot1_namespace
        ],
        output='screen'
    )
    
    # Robot 2 spawn
    robot2_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot2',
            '-file', os.path.join(turtlebot3_dir, 'models', 'turtlebot3_waffle_pi', 'model.sdf'),
            '-x', robot2_x,
            '-y', robot2_y,
            '-z', '0.01',
            '-robot_namespace', robot2_namespace
        ],
        output='screen'
    )
    
    # ← PUT THE NAV2 CODE HERE (between spawning and return statement)
    
    # Robot 1 navigation
    robot1_nav = GroupAction([
        PushRosNamespace(robot1_namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ]),
            launch_arguments={
                'namespace': robot1_namespace,
                'use_namespace': 'True',
                'params_file': os.path.join(dual_robot_nav_dir, 'config', 'robot1_nav2_params.yaml'),
                'use_sim_time': 'True'
            }.items()
        )
    ])
    
    # Robot 2 navigation
    robot2_nav = GroupAction([
        PushRosNamespace(robot2_namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ]),
            launch_arguments={
                'namespace': robot2_namespace,
                'use_namespace': 'True',
                'params_file': os.path.join(dual_robot_nav_dir, 'config', 'robot2_nav2_params.yaml'),
                'use_sim_time': 'True'
            }.items()
        )
    ])
    
    # Return everything together
    return LaunchDescription([
        gazebo,
        robot1_spawn,
        robot2_spawn,
        robot1_nav,
        robot2_nav
    ])

