#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def get_quaternion(yaw):
    """Convert yaw angle to quaternion for Nav2"""
    return {
        'qx': 0.0, 'qy': 0.0, 
        'qz': math.sin(yaw / 2.0), 
        'qw': math.cos(yaw / 2.0)
    }

class SmartFMSNavigator(Node):
    def __init__(self):
        super().__init__('smart_fms_navigator')
        self.navigator = BasicNavigator()
        self.current_position = None
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.get_logger().info("Professional FMS Navigator Initialized")

    def odom_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def navigate_to_waypoint(self, wp, is_station=True):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(wp['x'])
        goal_pose.pose.position.y = float(wp['y'])
        goal_pose.pose.orientation.z = wp['qz']
        goal_pose.pose.orientation.w = wp['qw']

        self.navigator.goToPose(goal_pose)
        
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            if is_station and wp.get('pause_time', 0) > 0:
                self.get_logger().info(f"⏸️ Pausing {wp['pause_time']}s at {wp['name']}")
                time.sleep(wp['pause_time'])
            return True
        return False

def main():
    rclpy.init()
    nav = SmartFMSNavigator()
    nav.navigator.waitUntilNav2Active()
    
    # Odometry safety check
    wait_count = 0
    while nav.current_position is None and wait_count < 20:
        rclpy.spin_once(nav, timeout_sec=1.0)
        wait_count += 1

    if nav.current_position is None:
        nav.get_logger().error("Critical: Odometry timeout.")
        return

    # Orientation Definitions
    # Robot faces "North" (toward machines) at 1.57 radians (90 degrees)
    north = get_quaternion(1.57) 
    EAST = get_quaternion(0.0)      # 0 radians
    SOUTH = get_quaternion(-1.57)   # -1.57 radians
    WEST = get_quaternion(3.14)     # 3.14 radians
    
    # 2.0 second timer for loading/unloading
    PAUSE = 3.0

    # --- COMPLETE MISSION WAYPOINTS ---
    # Follows the yellow floor path structure
    mission = [
        # --- CYCLE 1: MACHINE 1 (RED) ---
        {'name': 'L/U Station (Load M1)', 'x': -3.8, 'y': 0.0, **north, 'pause_time': PAUSE, 'is_station': True},
       
        
        {'name': ' Machine 1 (Unload)', 'x': -0.5, 'y': 2.0, **EAST, 'pause_time': PAUSE, 'is_station': True},
      
        {'name': 'L/U Station (Load M2)', 'x': -3.8, 'y': 0.0, **north, 'pause_time': PAUSE, 'is_station': True},


        # --- CYCLE 2: MACHINE 2 (BLUE) ---
       
        {'name': ' Machine 2 (Unload)', 'x': 3.5, 'y': 2.0, **EAST, 'pause_time': PAUSE, 'is_station': True},
        
        {'name': 'L/U Station (Load M3)', 'x': -3.8, 'y': 0.0, **north, 'pause_time': PAUSE, 'is_station': True},

        # --- CYCLE 3: MACHINE 3 (YELLOW) ---
       
        {'name': 'Machine 3 (Unload)', 'x': 3.5, 'y': -2.0, **SOUTH, 'pause_time': PAUSE, 'is_station': True},
       
        {'name': 'L/U Station (Load M4)', 'x': -3.8, 'y': 0.0, **north, 'pause_time': PAUSE, 'is_station': True},

        # --- CYCLE 4: MACHINE 4 (MAGENTA) ---
        
        {'name': 'Machine 4 (Unload)', 'x': -0.5, 'y': -2.0, **SOUTH, 'pause_time': PAUSE, 'is_station': True},
        
        

        # Return to Home Base

        {'name': 'Home Base', 'x': -3.8, 'y': 0.0, **north, 'pause_time': 0.0, 'is_station': True}
    ]

    # Sequential execution
    for wp in mission:
        nav.get_logger().info(f"🚀 Moving to {wp['name']}")
        if not nav.navigate_to_waypoint(wp, is_station=wp['is_station']):
            nav.get_logger().error(f"❌ Failed to reach {wp['name']}. Mission Aborted.")
            break

    nav.get_logger().info("🎉 FULL FACTORY SERVICE CYCLE COMPLETE")
    rclpy.shutdown()

if __name__ == '__main__':
    main()