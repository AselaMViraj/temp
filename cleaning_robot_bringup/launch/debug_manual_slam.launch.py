import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    bringup_pkg = get_package_share_directory('cleaning_robot_bringup')
    gazebo_pkg = get_package_share_directory('cleaning_robot_gazebo')
    nav_pkg = get_package_share_directory('cleaning_robot_navigation')
    desc_pkg = get_package_share_directory('cleaning_robot_description')

    # 1. Ignition Simulation
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg, 'launch', 'launch_ignition.launch.py')),
        launch_arguments={'world_file': 'cafe.world', 'slam': 'false'}.items()
    )

    # 2. SLAM Toolbox (FORCE LAUNCH)
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_pkg, 'launch', 'slam.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 3. RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(desc_pkg, 'rviz', 'view_robot.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        simulation,
        # Wait for sim then launch everything else
        TimerAction(period=5.0, actions=[slam_toolbox, rviz])
    ])
