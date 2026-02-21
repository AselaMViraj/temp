#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import qos_profile_sensor_data

class OdomTFPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Use Best Effort to match ros_gz_bridge default for some topics, 
        # ensuring we don't miss data if the bridge uses it.
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile_sensor_data
        )
        
        self.get_logger().info('Odometry TF Publisher started')
    
    def odom_callback(self, msg):
        try:
            t = TransformStamped()
            
            # Pass through the timestamp from the simulation
            t.header.stamp = msg.header.stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            
            # Copy position
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
            
            # Copy orientation
            t.transform.rotation = msg.pose.pose.orientation
            
            # Broadcast
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            self.get_logger().warn(f"Failed to publish TF: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OdomTFPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Node loop error: {e}")
    finally:
        # Destroy the node explicitly
        try:
            node.destroy_node()
        except:
            pass
        
        # Check if rclpy is still active before shutting down
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
