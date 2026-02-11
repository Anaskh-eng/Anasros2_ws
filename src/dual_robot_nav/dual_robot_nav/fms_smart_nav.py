#!/usr/bin/env python3
"""
Smart FMS Navigation Script
- Pauses at L/U and Machine stations
- Quick pass-through for intermediate waypoints
- Converts quaternion poses to navigation goals
"""

import time
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node


def quaternion_to_yaw(x, y, z, w):
    """Convert quaternion to yaw angle (theta)"""
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


class SmartFMSNavigator(Node):
    """
    Smart navigator that handles station pauses and quick waypoint transitions
    """
    
    def __init__(self):
        super().__init__('smart_fms_navigator')
        
        # Navigator
        self.navigator = BasicNavigator()
        
        # Current robot position
        self.current_position = None
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Proximity threshold (2cm = 0.02m)
        self.proximity_threshold = 0.02
        
        self.get_logger().info("Smart FMS Navigator initialized")
    
    def odom_callback(self, msg):
        """Store current robot position"""
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
    
    def get_distance_to_goal(self, goal_x, goal_y):
        """Calculate distance to goal"""
        if self.current_position is None:
            return float('inf')
        
        dx = goal_x - self.current_position[0]
        dy = goal_y - self.current_position[1]
        return math.sqrt(dx*dx + dy*dy)
    
    def create_goal_pose(self, x, y, qx, qy, qz, qw):
        """Create goal pose from position and quaternion"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw
        return goal_pose
    
    def navigate_to_waypoint(self, waypoint, is_station=False):
        """
        Navigate to a single waypoint
        
        Args:
            waypoint: dict with 'name', 'x', 'y', 'qx', 'qy', 'qz', 'qw', 'pause_time'
            is_station: if True, robot must reach exact position. If False, can skip when close
        """
        name = waypoint['name']
        x = waypoint['x']
        y = waypoint['y']
        qx = waypoint['qx']
        qy = waypoint['qy']
        qz = waypoint['qz']
        qw = waypoint['qw']
        pause_time = waypoint.get('pause_time', 0)
        
        # Create goal
        goal_pose = self.create_goal_pose(x, y, qx, qy, qz, qw)
        
        # Calculate yaw for display
        yaw = quaternion_to_yaw(qx, qy, qz, qw)
        
        self.get_logger().info(f"\n{'='*70}")
        self.get_logger().info(f"🎯 Navigating to: {name}")
        self.get_logger().info(f"   Position: x={x:.3f}, y={y:.3f}")
        self.get_logger().info(f"   Orientation: yaw={math.degrees(yaw):.1f}°")
        if pause_time > 0:
            self.get_logger().info(f"   ⏱️  Station pause: {pause_time}s")
        self.get_logger().info(f"{'='*70}")
        
        # Send goal
        self.navigator.goToPose(goal_pose)
        
        # Monitor navigation
        if is_station:
            # Station: Must reach exact position
            while not self.navigator.isTaskComplete():
                time.sleep(0.1)
                rclpy.spin_once(self, timeout_sec=0.1)
            
            result = self.navigator.getResult()
            
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"✅ Reached {name}")
                if pause_time > 0:
                    self.get_logger().info(f"⏸️  Pausing for {pause_time} seconds at station...")
                    time.sleep(pause_time)
                return True
            else:
                self.get_logger().error(f"❌ Failed to reach {name}")
                return False
        
        else:
            # Intermediate waypoint: Can skip when close
            while not self.navigator.isTaskComplete():
                distance = self.get_distance_to_goal(x, y)
                
                if distance <= self.proximity_threshold:
                    # Close enough! Cancel goal and move to next
                    self.get_logger().info(f"✓ Near {name} (within {self.proximity_threshold}m) - continuing")
                    self.navigator.cancelTask()
                    return True
                
                time.sleep(0.1)
                rclpy.spin_once(self, timeout_sec=0.1)
            
            result = self.navigator.getResult()
            
            if result == TaskResult.SUCCEEDED or result == TaskResult.CANCELED:
                self.get_logger().info(f"✓ Passed {name}")
                return True
            else:
                self.get_logger().error(f"❌ Navigation failed at {name}")
                return False


def main():
    rclpy.init()
    
    nav = SmartFMSNavigator()
    
    # Wait for Nav2
    print("\n🤖 Waiting for Nav2 to activate...")
    nav.navigator.waitUntilNav2Active()
    print("✓ Nav2 is ready!\n")
    
    # Wait for odometry
    while nav.current_position is None and rclpy.ok():
        print("⏳ Waiting for odometry data...")
        rclpy.spin_once(nav, timeout_sec=0.5)
    print("✓ Robot position received\n")
    
    # ========== FMS MISSION: PICKUP FROM L/U TO MACHINE 1 ==========
    # Based on your coordinates - Pickup first package and deliver to M1
    
    mission_waypoints = [
        # 1. Start beside L/U station (STATION - pause here)
        {
            'name': 'L/U Station - Pickup',
            'x': 0.20966120538047978,
            'y': -1.902255934339502,
            'qx': -0.0031592363572045287,
            'qy': -0.0001576172644466114,
            'qz': 0.9998741547011024,
            'qw': -0.015545711408395332,
            'pause_time': 5,  # Wait 5 seconds to load package
            'is_station': True
        },
        
        # 2. Intermediate waypoint (QUICK PASS)
        {
            'name': 'Waypoint 1',
            'x': -2.5870476330905325,
            'y': -1.0970750789586092,
            'qx': -0.003142167371664263,
            'qy': 8.93951275428286e-05,
            'qz': 0.9996221083085101,
            'qw': 0.02730859523243553,
            'pause_time': 0,
            'is_station': False
        },
        
        # 3. Intermediate waypoint (QUICK PASS)
        {
            'name': 'Waypoint 2',
            'x': -4.996368300777716,
            'y': -0.9643302439165773,
            'qx': -0.003198523351942905,
            'qy': 0.00010358846064775102,
            'qz': 0.9996163075056717,
            'qw': 0.02751356913460526,
            'pause_time': 0,
            'is_station': False
        },
        
        # 4. Intermediate waypoint (QUICK PASS)
        {
            'name': 'Waypoint 3',
            'x': -5.54712261538006,
            'y': 0.08530067554057702,
            'qx': -0.0022092723318812907,
            'qy': 0.002243528734619891,
            'qz': 0.7016715242494915,
            'qw': 0.7124936194465015,
            'pause_time': 0,
            'is_station': False
        },
        
        # 5. Machine M1 approach (STATION - pause here)
        {
            'name': 'Machine M1 - Delivery',
            'x': -3.6570490504822923,
            'y': 0.33671729591611893,
            'qx': -2.129343903921705e-05,
            'qy': -0.00311411572837932,
            'qz': -0.019722399304954015,
            'qw': -0.9998006445264355,
            'pause_time': 10,  # Wait 10 seconds to unload package
            'is_station': True
        }
    ]
    
    # =================================================================
    
    print("="*70)
    print("🏭 FMS MISSION: PICKUP & DELIVERY")
    print("="*70)
    print(f"Mission: L/U Station → Machine M1")
    print(f"Total waypoints: {len(mission_waypoints)}")
    print(f"  - Stations (pause): 2")
    print(f"  - Intermediate (pass): 3")
    print("="*70 + "\n")
    
    # Execute mission
    for i, waypoint in enumerate(mission_waypoints, 1):
        nav.get_logger().info(f"\n📍 Waypoint {i}/{len(mission_waypoints)}")
        
        success = nav.navigate_to_waypoint(
            waypoint,
            is_station=waypoint['is_station']
        )
        
        if not success:
            nav.get_logger().error("❌ Mission aborted due to navigation failure")
            break
        
        # Small delay between waypoints
        if i < len(mission_waypoints):
            time.sleep(0.5)
    
    print("\n" + "="*70)
    print("🎉 MISSION COMPLETE!")
    print("="*70)
    print("Package successfully delivered from L/U to Machine M1")
    print("="*70 + "\n")
    
    # Cleanup
    nav.navigator.lifecycleShutdown()
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
