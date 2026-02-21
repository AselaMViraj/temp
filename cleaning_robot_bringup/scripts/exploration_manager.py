#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from visualization_msgs.msg import MarkerArray
import time

class ExplorationManager(Node):
    def __init__(self):
        super().__init__('exploration_manager')
        
        self.start_pose = None
        self.exploration_active = True
        self.return_to_start_triggered = False
        self.zero_frontier_count = 0
        
        # Subscriber to frontiers to monitor exploration status
        self.frontiers_sub = self.create_subscription(
            MarkerArray,
            '/explore/frontiers',
            self.frontiers_callback,
            10
        )
        
        # Action client for Nav2 to return home
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publisher to current pose to record start position
        self.get_logger().info("Exploration Manager started. Monitoring frontiers...")

    def frontiers_callback(self, msg):
        if not self.exploration_active:
            return
            
        num_frontiers = len(msg.markers)
        
        if num_frontiers == 0:
            self.zero_frontier_count += 1
            if self.zero_frontier_count % 10 == 0:
                 self.get_logger().info(f"No frontiers seen for {self.zero_frontier_count} check intervals...")

            if self.zero_frontier_count > 30: # Increased to 30 to allow time for turning and re-evaluating
                self.get_logger().info("Persistent lack of frontiers detected! Exploration complete.")
                print("\n" + "="*50)
                print("   EXPLORATION COMPLETED   ")
                print("   Returning to Start Point (0,0)   ")
                print("="*50 + "\n")
                self.trigger_return_to_start()
                self.exploration_active = False
        else:
            if self.zero_frontier_count > 0:
                self.get_logger().debug(f"Frontiers found again ({num_frontiers}). Resetting completion count.")
            self.zero_frontier_count = 0

    def trigger_return_to_start(self):
        if self.return_to_start_triggered:
            return
            
        self.get_logger().info("Triggering return to start...")
        self.return_to_start_triggered = True
        
        # Wait for Nav2 action server
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available!")
            return

        # Prepare goal (assuming start is 0,0,0)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = 0.0
        goal_msg.pose.pose.position.y = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info("Sending goal: Return to start position (0,0)")
        self.nav_to_pose_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    manager = ExplorationManager()
    rclpy.spin(manager)
    manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
