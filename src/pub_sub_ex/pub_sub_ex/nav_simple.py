#!/usr/bin/env python3
"""
SIMPLE Nav2 Navigation Script
Minimal template for navigating through waypoints
"""

import time
import math
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy


def main():
    # Initialize ROS2
    rclpy.init()
    navigator = BasicNavigator()
    
    # Wait for Nav2 to activate
    print("Waiting for Nav2...")
    navigator.waitUntilNav2Active()
    print("Nav2 is ready!\n")
    
    # ========== YOUR WAYPOINTS HERE ==========
    # Change these to your actual coordinates from RViz
    goals = [
        (-2.190, 2.210, -2.775),   # (x, y, theta) in radians
        (-2.226, -0.332, -0.916),
        (6.434, -0.434, -0.060),
    ]
    # =========================================
    
    # Navigate to each goal
    for i, (x, y, theta) in enumerate(goals, 1):
        print(f"Goal {i}/{len(goals)}: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")
        
        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_pose.pose.orientation.w = math.cos(theta / 2.0)
        
        # Send goal
        navigator.goToPose(goal_pose)
        
        # Wait for completion
        while not navigator.isTaskComplete():
            time.sleep(0.1)
        
        # Check result
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"✓ Reached goal {i}!\n")
            time.sleep(2)  # Pause before next goal
        else:
            print(f"✗ Failed to reach goal {i}!")
            break
    
    print("Mission complete!")
    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
