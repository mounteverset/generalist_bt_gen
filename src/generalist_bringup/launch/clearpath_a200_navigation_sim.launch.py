#!/usr/bin/env python3
"""
Unified launch file for Clearpath A200 (Husky) Navigation Simulation
Launches: Gazebo Simulation + Nav2 + SLAM + RViz
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


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

    mock_gps_fix_topic_arg = DeclareLaunchArgument(
        'mock_gps_fix_topic',
        default_value='/gps/fix',
        description='Topic for the mock GPS NavSatFix publisher'
    )

    mock_gps_latitude_arg = DeclareLaunchArgument(
        'mock_gps_latitude_deg',
        default_value='48.284828555284605',
        description='Mock GPS latitude in decimal degrees'
    )

    mock_gps_longitude_arg = DeclareLaunchArgument(
        'mock_gps_longitude_deg',
        default_value='11.607701317621604',
        description='Mock GPS longitude in decimal degrees'
    )

    mock_gps_altitude_arg = DeclareLaunchArgument(
        'mock_gps_altitude_m',
        default_value='0.0',
        description='Mock GPS altitude in meters'
    )

    mock_gps_frame_id_arg = DeclareLaunchArgument(
        'mock_gps_frame_id',
        default_value='gps_link',
        description='Frame id for the mock GPS NavSatFix messages'
    )

    mock_gps_publish_rate_arg = DeclareLaunchArgument(
        'mock_gps_publish_rate_hz',
        default_value='1.0',
        description='Publish rate for the mock GPS NavSatFix messages'
    )

    # Launch configurations
    setup_path = LaunchConfiguration('setup_path')
    namespace = LaunchConfiguration('namespace')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    mock_gps_fix_topic = LaunchConfiguration('mock_gps_fix_topic')
    mock_gps_latitude = LaunchConfiguration('mock_gps_latitude_deg')
    mock_gps_longitude = LaunchConfiguration('mock_gps_longitude_deg')
    mock_gps_altitude = LaunchConfiguration('mock_gps_altitude_m')
    mock_gps_frame_id = LaunchConfiguration('mock_gps_frame_id')
    mock_gps_publish_rate = LaunchConfiguration('mock_gps_publish_rate_hz')

    # Clearpath generator scripts use /usr/bin/env python3 and require python3-apt.
    # Force system Python precedence so these scripts don't resolve to conda Python.
    prefer_system_python = SetEnvironmentVariable(
        name='PATH',
        value=[
            TextSubstitution(text='/usr/bin:'),
            EnvironmentVariable('PATH', default_value=''),
        ],
    )

    # Nav2's bt_navigator must link against the distro BehaviorTree.CPP ABI.
    # Prefer ROS distro libs over workspace overlays for this launch.
    prefer_ros_libs = SetEnvironmentVariable(
        name='LD_LIBRARY_PATH',
        value=[
            TextSubstitution(text='/opt/ros/jazzy/lib:'),
            EnvironmentVariable('LD_LIBRARY_PATH', default_value=''),
        ],
    )

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

    mock_gps_fix_publisher = Node(
        package='generalist_bringup',
        executable='mock_gps_fix_publisher_node',
        name='mock_gps_fix_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'topic_name': mock_gps_fix_topic,
            'frame_id': mock_gps_frame_id,
            'latitude_deg': mock_gps_latitude,
            'longitude_deg': mock_gps_longitude,
            'altitude_m': mock_gps_altitude,
            'publish_rate_hz': mock_gps_publish_rate,
        }],
    )

    return LaunchDescription([
        setup_path_arg,
        namespace_arg,
        world_arg,
        use_sim_time_arg,
        x_arg,
        y_arg,
        yaw_arg,
        mock_gps_fix_topic_arg,
        mock_gps_latitude_arg,
        mock_gps_longitude_arg,
        mock_gps_altitude_arg,
        mock_gps_frame_id_arg,
        mock_gps_publish_rate_arg,
        prefer_system_python,
        prefer_ros_libs,
        mock_gps_fix_publisher,
        simulation_launch,
        nav2_delayed,
        slam_delayed,
        rviz_delayed,
    ])
