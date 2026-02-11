import rclpy
import math
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import action_msgs.msg

def euler_to_quaternion(yaw):
    """Converts yaw angle to quaternion"""
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qx, qy, qz, qw

def quaternion_to_euler(qx, qy, qz, qw):
    """Convert quaternion to yaw angle"""
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

class NavGoalSender(Node):
    def __init__(self):
        super().__init__('nav_goal_sender')
        self.get_logger().info('Starting Navigation Goal Sender Node...')
        
        # Action client
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # Subscribe to odometry to check robot position
        self.current_pose = None
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Subscribe to velocity
        self.current_velocity = None
        self.vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10
        )
        
        self.goal_handle = None

    def odom_callback(self, msg):
        """Store current robot pose"""
        self.current_pose = msg.pose.pose

    def velocity_callback(self, msg):
        """Store current velocity"""
        self.current_velocity = msg

    def get_distance_to_goal(self, goal_x, goal_y):
        """Calculate distance from current position to goal"""
        if self.current_pose is None:
            return float('inf')
        
        dx = goal_x - self.current_pose.position.x
        dy = goal_y - self.current_pose.position.y
        return math.sqrt(dx*dx + dy*dy)

    def get_angle_difference(self, target_yaw):
        """Calculate angle difference from current orientation to target"""
        if self.current_pose is None:
            return float('inf')
        
        current_yaw = quaternion_to_euler(
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        )
        
        diff = target_yaw - current_yaw
        # Normalize to [-pi, pi]
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        
        return abs(diff)

    def is_near_goal(self, goal_x, goal_y, goal_theta, 
                     position_tolerance=0.20, angle_tolerance=0.35):
        """Check if robot is close enough to goal"""
        distance = self.get_distance_to_goal(goal_x, goal_y)
        angle_diff = self.get_angle_difference(goal_theta)
        
        return distance < position_tolerance and angle_diff < angle_tolerance

    def is_robot_stopped(self, threshold=0.02):
        """Check if robot has stopped moving"""
        if self.current_velocity is None:
            return False
        
        linear_vel = abs(self.current_velocity.linear.x)
        angular_vel = abs(self.current_velocity.angular.z)
        
        return linear_vel < threshold and angular_vel < threshold

    def send_goal(self, x, y, theta, position_tol=0.20, angle_tol=0.35):
        """
        Sends navigation goal and monitors progress.
        Cancels goal once robot is close enough to prevent oscillation.
        """
        self.get_logger().info('Waiting for action server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server /navigate_to_pose not available!')
            return False

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        qx, qy, qz, qw = euler_to_quaternion(theta)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        
        goal_msg.pose = pose

        # Send goal
        self.get_logger().info(f'Sending goal: (x={x:.2f}, y={y:.2f}, theta={theta:.2f})')
        goal_handle_future = self._action_client.send_goal_async(goal_msg)
        
        rclpy.spin_until_future_complete(self, goal_handle_future)
        self.goal_handle = goal_handle_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the server.')
            return False

        self.get_logger().info('Goal accepted! Monitoring progress...')

        # Monitor progress and cancel when close enough
        while True:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Check if close to goal
            if self.is_near_goal(x, y, theta, position_tol, angle_tol):
                distance = self.get_distance_to_goal(x, y)
                angle_diff = self.get_angle_difference(theta)
                
                self.get_logger().info(
                    f'✓ Close enough to goal! Distance: {distance:.3f}m, '
                    f'Angle diff: {math.degrees(angle_diff):.1f}°'
                )
                
                # Cancel the goal to stop Nav2 from oscillating
                self.get_logger().info('Canceling goal to prevent oscillation...')
                cancel_future = self.goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                
                # Wait for robot to stop
                self.get_logger().info('Waiting for robot to stop...')
                timeout = 5.0
                start_time = time.time()
                while time.time() - start_time < timeout:
                    rclpy.spin_once(self, timeout_sec=0.1)
                    if self.is_robot_stopped():
                        self.get_logger().info('Robot has stopped. ✓')
                        return True
                
                return True
            
            # Check if navigation completed or failed
            result_future = self.goal_handle.get_result_async()
            if result_future.done():
                status = result_future.result().status
                
                # If succeeded normally
                if status == action_msgs.msg.GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().info('Goal reached successfully! ✓')
                    return True
                
                # If aborted but we're close, that's OK
                if status in [action_msgs.msg.GoalStatus.STATUS_ABORTED, 
                             action_msgs.msg.GoalStatus.STATUS_CANCELED]:
                    if self.is_near_goal(x, y, theta, position_tol * 1.5, angle_tol * 1.5):
                        self.get_logger().info('Goal canceled but robot is near target. Success! ✓')
                        return True
                    else:
                        self.get_logger().error(f'Navigation failed with status: {status}')
                        return False

def main(args=None):
    rclpy.init(args=args)
    node = NavGoalSender()
    
    # Wait for odometry data
    node.get_logger().info('Waiting for odometry data...')
    while node.current_pose is None and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info('Odometry received! Ready to navigate.')

    # Define your three goals
    goals = [
        (-2.190, 2.210, -2.775),      # Goal 1
        (-2.226, -0.332, -0.916),     # Goal 2
        (6.434, -0.434, -0.060)       # Goal 3
    ]

    for i, (x, y, theta) in enumerate(goals):
        node.get_logger().info(f'\n{"="*60}')
        node.get_logger().info(f'📍 Navigating to Goal #{i+1}/{len(goals)}')
        node.get_logger().info(f'   Target: x={x:.2f}, y={y:.2f}, theta={theta:.2f}')
        node.get_logger().info(f'{"="*60}')
        
        success = node.send_goal(x, y, theta)
        
        if not success:
            node.get_logger().error('❌ Failed to reach goal. Aborting mission.')
            break
        
        # Pause before next goal
        if i < len(goals) - 1:
            node.get_logger().info('Pausing 2 seconds before next goal...\n')
            time.sleep(2.0)

    node.get_logger().info('\n' + '='*60)
    node.get_logger().info('🎉 All goals completed successfully!')
    node.get_logger().info('='*60)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()