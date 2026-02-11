import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

class NavigationGoalSender(Node):
    def _init_(self):
        super()._init_('navigation_goal_sender')
        self.navigator = BasicNavigator()

    def send_goal(self, x, y, yaw):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        # Set orientation (e.g., using a quaternion from yaw)
        # For simplicity, a direct assignment is shown here, but a proper conversion is needed.
        goal_pose.pose.orientation.w = 1.0 

        self.navigator.goToPose(goal_pose)
        self.navigator.waitUntilNav2Active() # Wait for Nav2 to become active

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            # Process feedback if needed
            self.get_logger().info(f'Distance remaining: {feedback.distance_remaining}')

        result = self.navigator.getResult()
        if result == BasicNavigator.TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif result == BasicNavigator.TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif result == BasicNavigator.TaskResult.FAILED:
            self.get_logger().info('Goal failed!')

def main(args=None):
    rclpy.init(args=args)
    node = NavigationGoalSender()
    node.send_goal(1.0, 2.0, 0.0) # Example goal
    rclpy.shutdown()

if __name__ == '__main__':
    main()