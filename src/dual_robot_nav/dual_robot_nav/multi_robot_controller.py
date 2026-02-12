#!/usr/bin/env python3
"""
Multi-Robot Navigation Controller

This script demonstrates how to send navigation goals to multiple robots
in a shared environment using ROS 2 DDS infrastructure.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Empty
import time


class MultiRobotController(Node):
    def __init__(self):
        super().__init__('multi_robot_controller')
        
        # Create action clients for both robots
        self.robot1_action_client = ActionClient(
            self, 
            NavigateToPose, 
            'robot1/navigate_to_pose'
        )
        
        self.robot2_action_client = ActionClient(
            self, 
            NavigateToPose, 
            'robot2/navigate_to_pose'
        )
        
        # Service client for resetting the simulation if needed
        self.reset_service = self.create_client(Empty, '/reset_simulation')

    def wait_for_action_server(self):
        """Wait for the navigation action servers to become available."""
        print("Waiting for robot1 navigation server...")
        self.robot1_action_client.wait_for_server()
        print("Robot1 navigation server available!")
        
        print("Waiting for robot2 navigation server...")
        self.robot2_action_client.wait_for_server()
        print("Robot2 navigation server available!")

    def send_goal_to_robot(self, robot_name, x, y, theta=0.0):
        """Send a navigation goal to a specific robot."""
        goal_msg = NavigateToPose.Goal()
        
        # Set the target pose
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Orientation (convert theta to quaternion)
        import math
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        goal_msg.pose.pose.orientation.w = cos_theta
        goal_msg.pose.pose.orientation.z = sin_theta
        
        # Select the appropriate action client
        if robot_name == 'robot1':
            action_client = self.robot1_action_client
        elif robot_name == 'robot2':
            action_client = self.robot2_action_client
        else:
            print(f"Unknown robot: {robot_name}")
            return None
            
        # Send the goal
        print(f"Sending goal to {robot_name}: ({x}, {y}, {theta})")
        return action_client.send_goal_async(goal_msg)

    def send_coordinated_goals(self):
        """Send coordinated navigation goals to both robots."""
        print("Sending coordinated goals to both robots...")
        
        # Send goals to both robots simultaneously
        future1 = self.send_goal_to_robot('robot1', 2.0, 1.0, 0.0)
        future2 = self.send_goal_to_robot('robot2', -2.0, -1.0, 0.0)
        
        return future1, future2


def main(args=None):
    rclpy.init(args=args)
    
    controller = MultiRobotController()
    
    try:
        # Wait for action servers
        controller.wait_for_action_server()
        
        # Send coordinated goals
        future1, future2 = controller.send_coordinated_goals()
        
        # Wait for both robots to reach their goals
        print("Waiting for robots to reach their goals...")
        
        # In a real application, you would handle the futures properly
        # For now, we'll just sleep and let the navigation run
        time.sleep(10)
        
        print("Navigation commands sent. Robots should be navigating to their goals.")
        print("Use Ctrl+C to stop the controller.")
        
        # Keep the node alive
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print("\nShutting down multi-robot controller...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()