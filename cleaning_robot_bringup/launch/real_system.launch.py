import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    bringup_pkg = get_package_share_directory('cleaning_robot_bringup')
    nav_pkg = get_package_share_directory('cleaning_robot_navigation')
    desc_pkg = get_package_share_directory('cleaning_robot_description')

    # Launch arguments
    # No world_file needed for real robot
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    slam = LaunchConfiguration('slam', default='true')
    explore = LaunchConfiguration('explore', default='false')
    localization = LaunchConfiguration('localization', default='false')
    map_file = LaunchConfiguration('map', default='')
    
    # ---------------------------------------------------------
    # 1. Real Robot Hardware Drivers
    # ---------------------------------------------------------
    
    # Base Control (ros2_control, hardware interface, robot_state_publisher)
    base_drivers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_pkg, 'launch', 'launch_robot.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items()
    )
    
    # Lidar Driver (RPLidar)
    lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_pkg, 'launch', 'rplidar.launch.py'))
    )
    
    # Laser Scan Filter
    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_filter',
        parameters=[
            os.path.join(bringup_pkg, 'config', 'laser_filter.yaml')
        ],
        remappings=[
            ('/scan', '/scan_raw'),
            ('/scan_filtered', '/scan')
        ]
    )

    # ---------------------------------------------------------
    # 2. Autonomous Exploration (includes SLAM and Nav2)
    # ---------------------------------------------------------
    exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_pkg, 'launch', 'exploration_launch.py')),
        condition=IfCondition(explore),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # ---------------------------------------------------------
    # 3. Manual Mode Definitions (Only if explore=false)
    # ---------------------------------------------------------
    
    # Manual SLAM
    manual_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_pkg, 'launch', 'slam.launch.py')),
        condition=IfCondition(PythonExpression(["'", slam, "' == 'true' and '", explore, "' == 'false'"])),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # Manual Navigation
    # Launch if (SLAM is true OR Localization is true) AND (Explore is false)
    manual_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_pkg, 'launch', 'navigation_manual.launch.py')),
        condition=UnlessCondition(explore),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # Manual Localization (AMCL)
    # Only if SLAM is false AND Localization is true AND Explore is false
    manual_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_pkg, 'launch', 'localization_launch.py')),
        condition=IfCondition(PythonExpression(["'", slam, "' == 'false' and '", localization, "' == 'true' and '", explore, "' == 'false'"])),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items()
    )

    # ---------------------------------------------------------
    # 4. Visualization & Tools
    # ---------------------------------------------------------

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(desc_pkg, 'rviz', 'view_robot.rviz')],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rviz),
        env={**dict(os.environ), 'MESA_GL_VERSION_OVERRIDE': '3.3'},
        output='screen'
    )

    # Web Server & Helpers
    web_server_nodes = [
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'use_sim_time': False}],
            output='screen'
        ),
        Node(
            package='cleaning_robot_bringup',
            executable='waypoint_manager.py',
            name='waypoint_manager',
            parameters=[{'use_sim_time': False}],
            output='screen'
        ),
        Node(
            package='cleaning_robot_bringup',
            executable='web_server.py',
            name='web_server',
            output='screen'
        )
    ]

    # Auto Kickstart for Exploration (Clears blind spot)
    auto_kickstart =  TimerAction(
        period=15.0,
        actions=[
            Node(
                package='cleaning_robot_bringup',
                executable='auto_kickstart.py',
                name='auto_kickstart',
                output='screen',
                condition=IfCondition(explore)
            )
        ]
    )


    return LaunchDescription([
        # Args
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('explore', default_value='false'),
        DeclareLaunchArgument('localization', default_value='false'),
        DeclareLaunchArgument('map', default_value=''),
        
        LogInfo(msg=["Starting Real Robot System... SLAM: ", slam, ", EXPLORE: ", explore]),

        # Drivers
        base_drivers,
        lidar_driver,
        scan_filter,

        # Tools
        *web_server_nodes,
        auto_kickstart,
        
        # Navigation Stack (Delayed slightly to allow hardware to settle)
        TimerAction(period=5.0, actions=[
            manual_slam,
            manual_navigation,
            manual_localization,
            exploration
        ]),
        
        TimerAction(period=8.0, actions=[rviz])
    ])
