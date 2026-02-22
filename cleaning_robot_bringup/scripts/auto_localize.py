#!/usr/bin/env python3
"""
auto_localize.py
Automatically calls AMCL global localization on startup (spreads particles 
across the entire map), then rotates the robot slowly so AMCL can converge
to the correct position without any manual '2D Pose Estimate' input.
"""
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
import time


class AutoLocalizeNode(Node):
    def __init__(self):
        super().__init__('auto_localize_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Auto-Localize: Calling global localization...')
        self._call_global_localization()

    def _call_global_localization(self):
        client = self.create_client(Empty, '/reinitialize_global_localization')
        
        # Wait for AMCL to be ready
        if not client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error('AMCL global localization service not available!')
            return

        # Call the service — spreads particles everywhere on the map
        future = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        self.get_logger().info('Global localization initialized! Spinning to help AMCL converge...')

        # Rotate slowly for 15 seconds so AMCL can match scan to map
        twist = Twist()
        twist.angular.z = 0.15  # slow spin
        spin_duration = 15.0
        start = time.time()
        while time.time() - start < spin_duration:
            self.cmd_pub.publish(twist)
            time.sleep(0.1)  # publish at 10Hz — no executor needed

        # Stop
        self.cmd_pub.publish(Twist())
        self.get_logger().info('Auto-localization spin complete. AMCL should now be converged.')
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = AutoLocalizeNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
