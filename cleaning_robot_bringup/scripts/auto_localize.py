#!/usr/bin/env python3
"""
auto_localize.py
After Nav2 starts, rotates the robot slowly in place so AMCL can gather
scan data from multiple angles and converge on the correct position.
Uses the fixed initial_pose from nav2_params.yaml (set_initial_pose: true).
NOTE: Does NOT call /reinitialize_global_localization — that crashes AMCL 
on Jetson due to memory pressure when spreading particles across a large map.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class AutoLocalizeNode(Node):
    def __init__(self):
        super().__init__('auto_localize_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Auto-Localize: Rotating to help AMCL converge...')
        self._spin_for_localization()

    def _spin_for_localization(self):
        # Rotate slowly for 15 seconds so AMCL accumulates scan data from
        # multiple angles and converges on the correct position
        twist = Twist()
        twist.angular.z = 0.15  # slow spin at calibrated speed
        spin_duration = 15.0
        start = time.time()

        while time.time() - start < spin_duration:
            self.cmd_pub.publish(twist)
            time.sleep(0.1)  # 10Hz

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
