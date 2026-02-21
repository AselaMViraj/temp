import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('cleaning_robot_description')
    
    # Only launch RViz, nothing else! 
    # The Jetson handles robot_state_publisher and joint_state_publisher.
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'view_bot.rviz')
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        node_rviz
    ])
