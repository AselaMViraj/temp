import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('cleaning_robot_bringup')
    nav_pkg = get_package_share_directory('cleaning_robot_navigation')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1. SLAM Toolbox (started via our custom slam.launch.py)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_pkg, 'launch', 'slam.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. Navigation 2 (started via our custom navigation_manual.launch.py)
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_pkg, 'launch', 'navigation_manual.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 3. Explore Lite
    explore_params = os.path.join(nav_pkg, 'config', 'explore_params.yaml')
    explore_lite = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        parameters=[explore_params, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 4. Exploration Manager (return home logic)
    exploration_manager = Node(
        package='cleaning_robot_bringup',
        executable='exploration_manager.py',
        name='exploration_manager',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        slam,
        # Give SLAM a moment to start before starting Nav2
        TimerAction(period=5.0, actions=[navigation]),
        # Give Nav2 a moment to start before starting exploration
        TimerAction(period=15.0, actions=[explore_lite, exploration_manager])
    ])
