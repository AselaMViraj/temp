import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, GroupAction, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    bringup_pkg = get_package_share_directory('cleaning_robot_bringup')
    gazebo_pkg = get_package_share_directory('cleaning_robot_gazebo')
    nav_pkg = get_package_share_directory('cleaning_robot_navigation')
    desc_pkg = get_package_share_directory('cleaning_robot_description')

    # Launch arguments
    world_file = LaunchConfiguration('world_file', default='cafe.world')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    slam = LaunchConfiguration('slam', default='true')
    explore = LaunchConfiguration('explore', default='false')
    localization = LaunchConfiguration('localization', default='false')
    map_file = LaunchConfiguration('map', default='')

    # Ignition Simulation
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg, 'launch', 'launch_ignition.launch.py')),
        launch_arguments={
            'world_file': world_file,
            'slam': 'false'
        }.items()
    )

    # Autonomous Exploration (includes SLAM and Nav2)
    exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_pkg, 'launch', 'exploration_launch.py')),
        condition=IfCondition(explore)
    )

    # Manual Mode Definitions (Only if explore=false)
    
    # 1. Manual SLAM
    manual_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_pkg, 'launch', 'slam.launch.py')),
        condition=IfCondition(PythonExpression(["'", slam, "' == 'true' and '", explore, "' == 'false'"])),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2. Manual Navigation
    # Launch if (SLAM is true OR Localization is true) AND (Explore is false)
    manual_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_pkg, 'launch', 'navigation_manual.launch.py')),
        condition=UnlessCondition(explore),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 3. Manual Localization (AMCL)
    # Only if SLAM is false AND Localization is true AND Explore is false
    manual_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_pkg, 'launch', 'localization_launch.py')),
        condition=IfCondition(PythonExpression(["'", slam, "' == 'false' and '", localization, "' == 'true' and '", explore, "' == 'false'"])),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true',
            'autostart': 'true'
        }.items()
    )

    # RViz - launching appropriately
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(desc_pkg, 'rviz', 'view_robot.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        env={**dict(os.environ), 'MESA_GL_VERSION_OVERRIDE': '3.3'},
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_file', default_value='cafe.world'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('explore', default_value='false'),
        DeclareLaunchArgument('localization', default_value='false'),
        DeclareLaunchArgument('map', default_value=''),
        
        simulation,

        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
        Node(
            package='cleaning_robot_bringup',
            executable='waypoint_manager.py',
            name='waypoint_manager',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
        Node(
            package='cleaning_robot_bringup',
            executable='web_server.py',
            name='web_server',
            output='screen'
        ),

        # Auto Kickstart for Exploration (Clears blind spot)
        TimerAction(
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
        ),

        # Staggered Launch parameters
        LogInfo(msg=["Launch Args - slam: ", slam, ", explore: ", explore]),
        
        # Execute Manual Ops immediately if condition is met
        TimerAction(period=5.0, actions=[
            manual_slam,
            manual_navigation,
            manual_localization
        ]),
        
        # Exploration handles its own timing or can be delayed slightly
        TimerAction(period=5.0, actions=[exploration]),
        
        TimerAction(period=10.0, actions=[rviz])
    ])
