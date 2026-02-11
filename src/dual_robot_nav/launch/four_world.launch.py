from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    dual_robot_nav_dir = get_package_share_directory('dual_robot_nav')
    turtlebot3_dir = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Define your 4 world files (change these to your actual world file names)
    world1 = os.path.join(dual_robot_nav_dir, 'worlds', 'fms_layout1.world')
    world2 = os.path.join(dual_robot_nav_dir, 'worlds', 'fms_layout2.world')
    world3 = os.path.join(dual_robot_nav_dir, 'worlds', 'fms_layout3.world')
    world4 = os.path.join(dual_robot_nav_dir, 'worlds', 'fms_layout4.world')
    
    # Model file for TurtleBot3 Waffle Pi
    model_file = os.path.join(turtlebot3_dir, 'models', 'turtlebot3_waffle_pi', 'model.sdf')
    
    # ========== GAZEBO INSTANCE 1 ==========
    gazebo1 = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world1
        ],
        output='screen',
        additional_env={'GAZEBO_MODEL_PATH': os.path.join(turtlebot3_dir, 'models')}
    )
    
    gzclient1 = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen',
        additional_env={'GAZEBO_MASTER_URI': 'http://localhost:11345'}
    )
    
    robot1_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='robot1',
        arguments=[
            '-entity', 'robot1',
            '-file', model_file,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
            '-robot_namespace', 'robot1'
        ],
        output='screen'
    )
    
    robot1_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'namespace': 'robot1',
            'use_namespace': 'True',
            'use_sim_time': 'True',
            'params_file': os.path.join(dual_robot_nav_dir, 'config', 'robot1_nav2_params.yaml'),
        }.items()
    )
    
    # ========== GAZEBO INSTANCE 2 ==========
    gazebo2 = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world2
        ],
        output='screen',
        additional_env={
            'GAZEBO_MODEL_PATH': os.path.join(turtlebot3_dir, 'models'),
            'GAZEBO_MASTER_URI': 'http://localhost:11346'
        }
    )
    
    gzclient2 = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen',
        additional_env={'GAZEBO_MASTER_URI': 'http://localhost:11346'}
    )
    
    robot2_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='robot2',
        arguments=[
            '-entity', 'robot2',
            '-file', model_file,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
            '-robot_namespace', 'robot2',
            '-gazebo_namespace', '/gazebo2'
        ],
        output='screen'
    )
    
    robot2_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'namespace': 'robot2',
            'use_namespace': 'True',
            'use_sim_time': 'True',
            'params_file': os.path.join(dual_robot_nav_dir, 'config', 'robot2_nav2_params.yaml'),
        }.items()
    )
    
    # ========== GAZEBO INSTANCE 3 ==========
    gazebo3 = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world3
        ],
        output='screen',
        additional_env={
            'GAZEBO_MODEL_PATH': os.path.join(turtlebot3_dir, 'models'),
            'GAZEBO_MASTER_URI': 'http://localhost:11347'
        }
    )
    
    gzclient3 = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen',
        additional_env={'GAZEBO_MASTER_URI': 'http://localhost:11347'}
    )
    
    robot3_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='robot3',
        arguments=[
            '-entity', 'robot3',
            '-file', model_file,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
            '-robot_namespace', 'robot3',
            '-gazebo_namespace', '/gazebo3'
        ],
        output='screen'
    )
    
    robot3_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'namespace': 'robot3',
            'use_namespace': 'True',
            'use_sim_time': 'True',
            'params_file': os.path.join(dual_robot_nav_dir, 'config', 'robot3_nav2_params.yaml'),
        }.items()
    )
    
    # ========== GAZEBO INSTANCE 4 ==========
    gazebo4 = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world4
        ],
        output='screen',
        additional_env={
            'GAZEBO_MODEL_PATH': os.path.join(turtlebot3_dir, 'models'),
            'GAZEBO_MASTER_URI': 'http://localhost:11348'
        }
    )
    
    gzclient4 = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen',
        additional_env={'GAZEBO_MASTER_URI': 'http://localhost:11348'}
    )
    
    robot4_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='robot4',
        arguments=[
            '-entity', 'robot4',
            '-file', model_file,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
            '-robot_namespace', 'robot4',
            '-gazebo_namespace', '/gazebo4'
        ],
        output='screen'
    )
    
    robot4_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'namespace': 'robot4',
            'use_namespace': 'True',
            'use_sim_time': 'True',
            'params_file': os.path.join(dual_robot_nav_dir, 'config', 'robot4_nav2_params.yaml'),
        }.items()
    )
    
    return LaunchDescription([
        # Gazebo World 1
        gazebo1,
        gzclient1,
        robot1_spawn,
        robot1_nav,
        
        # Gazebo World 2
        gazebo2,
        gzclient2,
        robot2_spawn,
        robot2_nav,
        
        # Gazebo World 3
        gazebo3,
        gzclient3,
        robot3_spawn,
        robot3_nav,
        
        # Gazebo World 4
        gazebo4,
        gzclient4,
        robot4_spawn,
        robot4_nav,
    ])