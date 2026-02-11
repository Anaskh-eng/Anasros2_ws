# mission_planner/mission_planner/dual_robot_commander.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class DualRobotCommander(Node):
    def __init__(self):
        super().__init__('dual_robot_commander')
        
        # Action clients for both robots
        self.robot1_client = ActionClient(
            self, NavigateToPose, '/robot1/navigate_to_pose'
        )
        self.robot2_client = ActionClient(
            self, NavigateToPose, '/robot2/navigate_to_pose'
        )
        
    def send_goal_robot1(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'robot1/map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.robot1_client.wait_for_server()
        self.robot1_client.send_goal_async(goal_msg)
        self.get_logger().info(f'Sent goal to Robot1: ({x}, {y})')
        
    def send_goal_robot2(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'robot2/map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.robot2_client.wait_for_server()
        self.robot2_client.send_goal_async(goal_msg)
        self.get_logger().info(f'Sent goal to Robot2: ({x}, {y})')

def main(args=None):
    rclpy.init(args=args)
    commander = DualRobotCommander()
    
    # Example: Send goals to both robots
    commander.send_goal_robot1(3.0, 3.0)
    commander.send_goal_robot2(-2.0, 2.0)
    
    rclpy.spin(commander)
    commander.destroy_node()
    rclpy.shutdown()