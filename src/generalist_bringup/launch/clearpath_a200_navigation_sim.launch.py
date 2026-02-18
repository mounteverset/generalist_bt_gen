#!/usr/bin/env python3
"""
Unified launch file for Clearpath A200 (Husky) Navigation Simulation
Launches: Gazebo Simulation + Nav2 + SLAM + RViz
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    setup_path_arg = DeclareLaunchArgument(
        'setup_path',
        default_value='/home/luke/clearpath',
        description='Path to the robot.yaml configuration file directory'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='a200_0000',
        description='Robot namespace'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='warehouse',
        description='Gazebo world to load (warehouse, office, construction, etc.)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='X position of robot spawn'
    )

    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Y position of robot spawn'
    )

    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Yaw orientation of robot spawn'
    )

    # Launch configurations
    setup_path = LaunchConfiguration('setup_path')
    namespace = LaunchConfiguration('namespace')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    # 1. Launch Gazebo Simulation
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('clearpath_gz'),
                'launch',
                'simulation.launch.py'
            ])
        ),
        launch_arguments=[
            ('setup_path', setup_path),
            ('world', world),
            ('x', x),
            ('y', y),
            ('yaw', yaw),
        ]
    )

    # 2. Launch Nav2 (delayed 5 seconds to let simulation start)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('clearpath_nav2_demos'),
                'launch',
                'nav2.launch.py'
            ])
        ),
        launch_arguments=[
            ('setup_path', setup_path),
            ('use_sim_time', use_sim_time),
        ]
    )

    # Delay Nav2 launch by 5 seconds
    nav2_delayed = TimerAction(
        period=5.0,
        actions=[nav2_launch]
    )

    # 3. Launch SLAM (delayed 10 seconds to let Nav2 start)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('clearpath_nav2_demos'),
                'launch',
                'slam.launch.py'
            ])
        ),
        launch_arguments=[
            ('setup_path', setup_path),
            ('use_sim_time', use_sim_time),
        ]
    )

    slam_delayed = TimerAction(
        period=10.0,
        actions=[slam_launch]
    )

    # 4. Launch RViz (delayed 15 seconds for everything else)
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('clearpath_viz'),
                'launch',
                'view_navigation.launch.py'
            ])
        ),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time),
        ]
    )

    rviz_delayed = TimerAction(
        period=15.0,
        actions=[rviz_launch]
    )

    return LaunchDescription([
        setup_path_arg,
        namespace_arg,
        world_arg,
        use_sim_time_arg,
        x_arg,
        y_arg,
        yaw_arg,
        simulation_launch,
        nav2_delayed,
        slam_delayed,
        rviz_delayed,
    ])
