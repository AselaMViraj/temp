#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class KickstartNode(Node):
    def __init__(self):
        super().__init__('kickstart_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.start_time = time.time()
        self.run_duration = 3.0  # Drive for 3 seconds
        self.speed = 0.3
        self.kicked = False
        self.get_logger().info('Kickstart Node Started: Driving forward to clear blind spot...')

    def timer_callback(self):
        current_time = time.time()
        elapsed = current_time - self.start_time

        msg = Twist()
        
        # Wait 5 seconds before starting (let SLAM/Nav initialize)
        if elapsed < 8.0:
            self.get_logger().info(f'Waiting for system... {8.0 - elapsed:.1f}s')
            return

        # Drive forward for run_duration
        if elapsed < (8.0 + self.run_duration):
            msg.linear.x = self.speed
            self.publisher_.publish(msg)
            self.get_logger().info('KICKSTART: Moving forward...')
            self.kicked = True
        else:
            # Stop and shutdown
            msg.linear.x = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info('Kickstart complete. Stopping.')
            # Allow one more publish to ensure stop
            time.sleep(0.5)
            self.publisher_.publish(msg)
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = KickstartNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
