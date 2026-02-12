# dual_robot_nav/launch/dual_robot_navigation.launch.py

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get package directories
    dual_robot_nav_dir = get_package_share_directory('dual_robot_nav')
    turtlebot3_dir = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    
    # Robot parameters
    robot1_x = LaunchConfiguration('robot1_x', default='0.0')
    robot1_y = LaunchConfiguration('robot1_y', default='0.0')
    robot1_yaw = LaunchConfiguration('robot1_yaw', default='0.0')
    robot1_namespace = LaunchConfiguration('robot1_namespace', default='robot1')
    
    robot2_x = LaunchConfiguration('robot2_x', default='1.0')
    robot2_y = LaunchConfiguration('robot2_y', default='0.0')
    robot2_yaw = LaunchConfiguration('robot2_yaw', default='0.0')
    robot2_namespace = LaunchConfiguration('robot2_namespace', default='robot2')
    
    # World file
    world_file = LaunchConfiguration('world', default=os.path.join(dual_robot_nav_dir, 'worlds', 'fms_layout2.world'))
    
    # Map file for both robots
    map_file = LaunchConfiguration('map', default=os.path.join(dual_robot_nav_dir, 'maps', 'fms_layout2.yaml'))
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )
    
    # Static transform publisher to share the same map frame for both robots
    static_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_tf',
        arguments=['--frame-id', 'map', '--child-frame-id', 'robot1/map'],
        parameters=[{'use_sim_time': use_sim_time}]
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
            '-Y', robot1_yaw
        ],
        parameters=[{'use_sim_time': use_sim_time}],
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
            '-Y', robot2_yaw
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Robot 1 navigation stack with namespace
    robot1_group = GroupAction(
        actions=[
            PushRosNamespace(robot1_namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': os.path.join(dual_robot_nav_dir, 'config', 'robot1_nav2_params.yaml')
                }.items()
            ),
        ]
    )
    
    # Robot 2 navigation stack with namespace
    robot2_group = GroupAction(
        actions=[
            PushRosNamespace(robot2_namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': os.path.join(dual_robot_nav_dir, 'config', 'robot2_nav2_params.yaml')
                }.items()
            ),
        ]
    )
    
    # Shared map server for both robots
    shared_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='shared_map_server',
        parameters=[
            {'yaml_filename': map_file},
            {'topic': 'map'},
            {'frame_id': 'map'},
            {'use_sim_time': use_sim_time},
            {'always_send_updates': True}
        ],
        output='screen'
    )
    
    # Map client for robot1 to access shared map
    robot1_map_client = Node(
        package='nav2_map_server',
        executable='map_client',
        name='robot1_map_client',
        namespace='robot1',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/map', '/map')]
    )
    
    # Map client for robot2 to access shared map
    robot2_map_client = Node(
        package='nav2_map_server',
        executable='map_client',
        name='robot2_map_client',
        namespace='robot2',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/map', '/map')]
    )
    
    # Lifecycle manager for shared map server
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_maps',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['shared_map_server']}
        ]
    )
    
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('autostart', default_value='True', description='Automatically startup the nav2 stack'),
        
        # Gazebo
        gazebo,
        
        # Static transforms
        static_map_tf,
        
        # Robot spawners
        robot1_spawn,
        robot2_spawn,
        
        # Shared map server
        shared_map_server,
        robot1_map_client,
        robot2_map_client,
        lifecycle_manager,
        
        # Robot navigation stacks
        robot1_group,
        robot2_group,
    ])

