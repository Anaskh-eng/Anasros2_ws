# Multi-Robot Navigation with DDS and Shared Maps in ROS 2

## Overview

This document explains how to implement a multi-robot navigation system using ROS 2's Data Distribution Service (DDS) infrastructure. The system consists of multiple TurtleBot3 robots operating in a shared environment with a common map.

## DDS Concepts for Multi-Robot Systems

### What is DDS?

Data Distribution Service (DDS) is a middleware standard that provides scalable, real-time data exchange between distributed systems. In ROS 2, DDS implementations (like Fast DDS, Cyclone DDS) handle communication between nodes across different robots.

### DDS Architecture Benefits for Multi-Robot Systems

1. **Decentralized Communication**: Each robot can communicate directly without a central master
2. **Robust Discovery**: Robots can discover each other automatically
3. **Real-time Performance**: Low-latency communication critical for navigation
4. **Scalability**: Easy to add more robots to the system
5. **Fault Tolerance**: Failure of one robot doesn't bring down the entire system

### Topics and Services in Multi-Robot Navigation

Each robot maintains its own namespace to avoid topic conflicts:

- `/robot1/scan` - Laser scan data for robot 1
- `/robot2/scan` - Laser scan data for robot 2
- `/robot1/cmd_vel` - Velocity commands for robot 1
- `/robot2/cmd_vel` - Velocity commands for robot 2
- `/robot1/amcl_pose` - AMCL pose estimate for robot 1
- `/robot2/amcl_pose` - AMCL pose estimate for robot 2

## System Architecture

### Components

1. **Gazebo Simulator**: Provides physics simulation and sensor data
2. **Robot Spawning**: Each robot is spawned with unique position and namespace
3. **Navigation Stacks**: Independent navigation stack for each robot
4. **Shared Map Server**: Single map accessible by all robots
5. **Transform Management**: Coordinate frames for multi-robot localization

### Coordinate Frame Management

In our implementation:
- Global map frame: `map`
- Robot 1 frames: `robot1/odom`, `robot1/base_link`, etc.
- Robot 2 frames: `robot2/odom`, `robot2/base_link`, etc.

The static transform publisher ensures all robots reference the same global map.

### Parameter Configuration

Each robot uses its own parameter file:
- `robot1_nav2_params.yaml` - Parameters for robot 1 navigation
- `robot2_nav2_params.yaml` - Parameters for robot 2 navigation

Both configurations reference the same global map frame while maintaining robot-specific costmaps and localization.

## Launch File Explanation

Our `dual_robot_navigation.launch.py` implements:

1. **Gazebo Launch**: Starts the simulator with a custom world
2. **Robot Spawning**: Places robots at specified coordinates in Gazebo
3. **Map Server**: Provides a shared map to all robots
4. **Navigation Stacks**: Launches independent navigation for each robot
5. **Transform Management**: Ensures proper coordinate frame relationships

## Key Features of Our Implementation

### Shared Map Access
- Single map server serves the same map to both robots
- Each robot maintains its own localization relative to the shared map
- Costmaps are robot-local but reference the global map frame

### Independent Navigation
- Each robot plans its own path independently
- Collision avoidance is handled through local costmaps
- Goal positions can be sent to individual robots without affecting others

### Robust Communication
- DDS handles message routing between namespaces automatically
- Topic remapping ensures proper inter-robot communication
- Lifecycle management ensures proper startup sequence

## Running the System

To launch the multi-robot navigation system:

```bash
ros2 launch dual_robot_nav dual_robot_navigation.launch.py
```

To send navigation goals to individual robots:

```bash
# For robot 1
ros2 action send_goal /robot1/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}, header: {frame_id: 'map'}}}"

# For robot 2
ros2 action send_goal /robot2/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: -1.0, y: -1.0, z: 0.0}, orientation: {w: 1.0}}, header: {frame_id: 'map'}}}"
```

## Troubleshooting

### Common Issues
1. **Topic Conflicts**: Ensure each robot has a unique namespace
2. **TF Errors**: Check that all transforms are properly published
3. **Map Not Loading**: Verify the map file path and format
4. **Navigation Failing**: Check costmap parameters and obstacle detection

### Debugging Commands
```bash
# List all topics
ros2 topic list

# Check TF tree
ros2 run tf2_tools view_frames

# Visualize in RViz
rviz2
```

## Extending the System

This implementation can be extended to include:
- More robots by adding additional namespaces and parameters
- Robot coordination algorithms
- Communication protocols between robots
- Dynamic obstacle avoidance
- Task allocation systems

## Conclusion

This multi-robot navigation system demonstrates how ROS 2's DDS infrastructure enables complex robotic systems with multiple autonomous agents sharing a common environment. The modular design allows for easy scaling and customization based on specific application requirements.