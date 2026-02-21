#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
import json
import os
import math
from ament_index_python.packages import get_package_share_directory

class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')
        
        # Path to storage file
        self.declare_parameter('waypoints_file', 'waypoints.json')
        filename = self.get_parameter('waypoints_file').get_parameter_value().string_value
        
        home_path = os.path.expanduser('~/.cleaning_robot')
        os.makedirs(home_path, exist_ok=True)
        self.storage_path = os.path.join(home_path, filename)
            
        self.waypoints = {}
        self.current_costmap = None
        self.load_waypoints()
        
        # Action Client for Nav2
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers
        self.waypoints_pub = self.create_publisher(String, 'waypoints_list', 10)
        
        # Subscribers
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10
        )
        self.add_waypoint_sub = self.create_subscription(
            String,
            'add_waypoint',
            self.add_waypoint_callback,
            10
        )
        self.nav_to_sub = self.create_subscription(
            String,
            'nav_to_waypoint',
            self.nav_to_waypoint_callback,
            10
        )
        self.reset_sub = self.create_subscription(
            String,
            'reset_waypoints',
            self.reset_callback,
            10
        )
        
        # Timer to periodically publish the list
        self.timer = self.create_timer(2.0, self.publish_waypoints)
        
        self.get_logger().info(f'Waypoint Manager started. Storing in: {self.storage_path}')

    def costmap_callback(self, msg):
        self.current_costmap = msg

    def load_waypoints(self):
        if os.path.exists(self.storage_path):
            try:
                with open(self.storage_path, 'r') as f:
                    self.waypoints = json.load(f)
                self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
            except Exception as e:
                self.get_logger().error(f'Failed to load waypoints: {e}')

    def save_waypoints(self):
        try:
            os.makedirs(os.path.dirname(self.storage_path), exist_ok=True)
            with open(self.storage_path, 'w') as f:
                json.dump(self.waypoints, f, indent=4)
        except Exception as e:
            self.get_logger().error(f'Failed to save waypoints: {e}')

    def world_to_grid(self, x, y):
        if not self.current_costmap:
            return None, None
        
        origin_x = self.current_costmap.info.origin.position.x
        origin_y = self.current_costmap.info.origin.position.y
        resolution = self.current_costmap.info.resolution
        
        gx = int((x - origin_x) / resolution)
        gy = int((y - origin_y) / resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        if not self.current_costmap:
            return None, None
            
        origin_x = self.current_costmap.info.origin.position.x
        origin_y = self.current_costmap.info.origin.position.y
        resolution = self.current_costmap.info.resolution
        
        wx = (gx * resolution) + origin_x + (resolution / 2.0)
        wy = (gy * resolution) + origin_y + (resolution / 2.0)
        return wx, wy

    def get_cost(self, gx, gy):
        if not self.current_costmap:
            return 255
        
        width = self.current_costmap.info.width
        height = self.current_costmap.info.height
        
        if gx < 0 or gy < 0 or gx >= width or gy >= height:
            return 255
            
        return self.current_costmap.data[gy * width + gx]

    def find_nearest_safe(self, x, y):
        if not self.current_costmap:
            return x, y
            
        gx, gy = self.world_to_grid(x, y)
        if self.get_cost(gx, gy) < 100: # Threshold for "safe enough"
            return x, y
            
        self.get_logger().info(f'Point ({x}, {y}) is on obstacle. Searching for safe spot...')
        
        # Spiral search
        for radius in range(1, 20): # Search up to 20 cells (~1 meter)
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                        
                    if self.get_cost(gx + dx, gy + dy) < 50: # Prefer very clear space
                        nx, ny = self.grid_to_world(gx + dx, gy + dy)
                        self.get_logger().info(f'Found safe spot at ({nx}, {ny})')
                        return nx, ny
                        
        return x, y

    def add_waypoint_callback(self, msg):
        try:
            parts = msg.data.split(',')
            if len(parts) == 5:
                name = parts[0]
                tx, ty = float(parts[1]), float(parts[2])
                
                # Check for safety
                safe_x, safe_y = self.find_nearest_safe(tx, ty)
                
                self.waypoints[name] = {
                    'x': safe_x,
                    'y': safe_y,
                    'z': float(parts[3]),
                    'w': float(parts[4])
                }
                self.save_waypoints()
                self.get_logger().info(f'Added waypoint: {name} (Snapped: {safe_x != tx})')
            else:
                self.get_logger().warn('Invalid add_waypoint format.')
        except Exception as e:
            self.get_logger().error(f'Error adding waypoint: {e}')

    def nav_to_waypoint_callback(self, msg):
        name = msg.data
        if name in self.waypoints:
            wp = self.waypoints[name]
            self.send_goal(wp['x'], wp['y'], wp['z'], wp['w'])

    def reset_callback(self, msg):
        self.waypoints = {}
        self.save_waypoints()
        self.publish_waypoints()
        self.get_logger().info('All waypoints reset.')

    def send_goal(self, x, y, z, w):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z
        goal_msg.pose.pose.orientation.w = w
        
        self.nav_action_client.wait_for_server()
        self.nav_action_client.send_goal_async(goal_msg)

    def publish_waypoints(self):
        msg = String()
        msg.data = json.dumps(self.waypoints)
        self.waypoints_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
