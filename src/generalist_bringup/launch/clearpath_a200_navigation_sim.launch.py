#!/usr/bin/env python3
"""
Unified launch file for Clearpath A200 (Husky) GPS Navigation Simulation
Launches: Gazebo Simulation + GPS localization + Nav2 + RViz
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from nav2_common.launch import RewrittenYaml


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
        'world_file',
        default_value='/home/luke/generalist_bt_gen/evaluation/Eching/Eching.world',
        description='Gazebo world file to load'
    )

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='Eching',
        description='World name declared inside the Gazebo world file'
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

    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.3',
        description='Z position of robot spawn'
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
        default_value='48.2841528016946',
        description='Mock GPS latitude in decimal degrees'
    )

    mock_gps_longitude_arg = DeclareLaunchArgument(
        'mock_gps_longitude_deg',
        default_value='11.608142615762631',
        description='Mock GPS longitude in decimal degrees'
    )

    mock_gps_altitude_arg = DeclareLaunchArgument(
        'mock_gps_altitude_m',
        default_value='471.40000000000146',
        description='Mock GPS altitude in meters'
    )

    mock_gps_frame_id_arg = DeclareLaunchArgument(
        'mock_gps_frame_id',
        default_value='base_link',
        description='Frame id for the mock GPS NavSatFix messages'
    )

    mock_gps_publish_rate_arg = DeclareLaunchArgument(
        'mock_gps_publish_rate_hz',
        default_value='1.0',
        description='Publish rate for the mock GPS NavSatFix messages'
    )

    gps_navigation_odom_topic_arg = DeclareLaunchArgument(
        'gps_navigation_odom_topic',
        default_value='/a200_0000/platform/odom/global',
        description='World-referenced odometry input used for GPS coordinate conversion'
    )

    enable_gps_navigation_arg = DeclareLaunchArgument(
        'enable_gps_navigation',
        default_value='true',
        choices=['true', 'false'],
        description=(
            'Launch GPS map localization and navsat_transform_node for Nav2 GPS goals. '
            'The simulation assumes odom yaw is ENU/world-referenced.'
        )
    )

    # Launch configurations
    setup_path = LaunchConfiguration('setup_path')
    namespace = LaunchConfiguration('namespace')
    world_file = LaunchConfiguration('world_file')
    world_name = LaunchConfiguration('world_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    mock_gps_fix_topic = LaunchConfiguration('mock_gps_fix_topic')
    mock_gps_latitude = LaunchConfiguration('mock_gps_latitude_deg')
    mock_gps_longitude = LaunchConfiguration('mock_gps_longitude_deg')
    mock_gps_altitude = LaunchConfiguration('mock_gps_altitude_m')
    mock_gps_frame_id = LaunchConfiguration('mock_gps_frame_id')
    mock_gps_publish_rate = LaunchConfiguration('mock_gps_publish_rate_hz')
    gps_navigation_odom_topic = LaunchConfiguration('gps_navigation_odom_topic')
    enable_gps_navigation = LaunchConfiguration('enable_gps_navigation')

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

    # This launch starts more processes than CycloneDDS' default auto-index limit.
    cyclone_participant_limit = SetEnvironmentVariable(
        name='CYCLONEDDS_URI',
        value=(
            '<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex>'
            '<MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>'
            '</Discovery></Domain></CycloneDDS>'
        ),
    )

    package_share_paths = ':'.join(
        os.path.join(prefix, 'share')
        for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(':')
        if prefix
    )
    gazebo_resources = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            TextSubstitution(text='/home/luke/generalist_bt_gen/evaluation/Eching:'),
            PathJoinSubstitution([FindPackageShare('clearpath_gz'), 'worlds']),
            TextSubstitution(text=':'),
            PathJoinSubstitution([FindPackageShare('clearpath_gz'), 'meshes']),
            TextSubstitution(text=f':{package_share_paths}:'),
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
        ],
    )

    # Clearpath's simulation wrapper only accepts its built-in *.sdf worlds.
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments=[
            ('gz_args', [
                world_file,
                TextSubstitution(text=' -r -v 4 --gui-config '),
                PathJoinSubstitution([
                    FindPackageShare('clearpath_gz'), 'config', 'gui.config'
                ]),
            ]),
        ]
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    robot_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('clearpath_gz'),
                'launch',
                'robot_spawn.launch.py'
            ])
        ),
        launch_arguments=[
            ('setup_path', setup_path),
            ('use_sim_time', use_sim_time),
            ('world', world_name),
            ('rviz', 'false'),
            ('x', x),
            ('y', y),
            ('z', z),
            ('yaw', yaw),
        ]
    )

    gps_fix_remap = SetRemap(
        src=[
            TextSubstitution(text='/'),
            namespace,
            TextSubstitution(text='/sensors/gps_0/fix'),
        ],
        dst='/gps/fix_raw',
    )

    gps_covariance = Node(
        package='generalist_bringup',
        executable='gps_covariance_node',
        name='gps_covariance',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/gps/fix_raw',
            'output_topic': mock_gps_fix_topic,
            'position_stddev_m': 0.2,
        }],
    )

    nav2_parameters = RewrittenYaml(
        source_file=PathJoinSubstitution([
            FindPackageShare('clearpath_nav2_demos'),
            'config',
            'a200',
            'nav2.yaml',
        ]),
        param_rewrites={
            'topic': [
                TextSubstitution(text='/'),
                namespace,
                TextSubstitution(text='/sensors/lidar2d_0/scan'),
            ],
            'local_costmap.local_costmap.ros__parameters.static_layer.enabled': 'false',
            'global_costmap.global_costmap.ros__parameters.rolling_window': 'true',
            'global_costmap.global_costmap.ros__parameters.width': '200',
            'global_costmap.global_costmap.ros__parameters.height': '200',
            'global_costmap.global_costmap.ros__parameters.track_unknown_space': 'false',
            'global_costmap.global_costmap.ros__parameters.static_layer.enabled': 'false',
            'waypoint_follower.ros__parameters.global_frame_id': 'map',
        },
        convert_types=True,
    )

    # GPS mode: Nav2 plans in a rolling obstacle costmap and does not require /map.
    nav2_launch = GroupAction(actions=[
        PushRosNamespace(namespace),
        SetRemap(
            src=[TextSubstitution(text='/'), namespace, TextSubstitution(text='/odom')],
            dst=[
                TextSubstitution(text='/'),
                namespace,
                TextSubstitution(text='/platform/odom'),
            ],
        ),
        # Generalist uses root-level Nav2 action names on the real robot.
        SetRemap(
            src=[
                TextSubstitution(text='/'),
                namespace,
                TextSubstitution(text='/navigate_to_pose'),
            ],
            dst='/navigate_to_pose',
        ),
        SetRemap(
            src=[
                TextSubstitution(text='/'),
                namespace,
                TextSubstitution(text='/follow_gps_waypoints'),
            ],
            dst='/follow_gps_waypoints',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py',
                ])
            ),
            launch_arguments=[
                ('namespace', namespace),
                ('use_sim_time', use_sim_time),
                ('params_file', nav2_parameters),
                ('use_composition', 'False'),
            ],
        ),
    ])

    # Delay Nav2 launch by 5 seconds
    nav2_delayed = TimerAction(
        period=5.0,
        actions=[nav2_launch]
    )

    # 3. Launch RViz (delayed 15 seconds for everything else)
    rviz_launch = GroupAction(actions=[
        SetRemap(
            src=[
                TextSubstitution(text='/'),
                namespace,
                TextSubstitution(text='/navigate_to_pose'),
            ],
            dst='/navigate_to_pose',
        ),
        SetRemap(
            src=[
                TextSubstitution(text='/'),
                namespace,
                TextSubstitution(text='/follow_gps_waypoints'),
            ],
            dst='/follow_gps_waypoints',
        ),
        IncludeLaunchDescription(
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
        ),
    ])

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

    # The Clearpath EKF remains the sole odom -> base_link publisher. This
    # second EKF fuses GPS position and publishes the only map -> odom edge.
    gps_global_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        namespace=namespace,
        name='ekf_map_node',
        output='screen',
        condition=IfCondition(enable_gps_navigation),
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('generalist_bringup'),
                'config',
                'gps_global_ekf_params.yaml',
            ]),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('odometry/filtered', 'platform/odom/global'),
        ],
    )

    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        condition=IfCondition(enable_gps_navigation),
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('generalist_bringup'),
                'config',
                'navsat_transform_params.yaml',
            ]),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/tf', [TextSubstitution(text='/'), namespace, TextSubstitution(text='/tf')]),
            ('/tf_static', [
                TextSubstitution(text='/'), namespace, TextSubstitution(text='/tf_static')
            ]),
            ('gps/fix', mock_gps_fix_topic),
            ('odometry/filtered', gps_navigation_odom_topic),
            ('odometry/gps', '/a200_0000/platform/odom/gps'),
            ('gps/filtered', '/gps/filtered'),
        ],
    )

    return LaunchDescription([
        setup_path_arg,
        namespace_arg,
        world_arg,
        world_name_arg,
        use_sim_time_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        mock_gps_fix_topic_arg,
        mock_gps_latitude_arg,
        mock_gps_longitude_arg,
        mock_gps_altitude_arg,
        mock_gps_frame_id_arg,
        mock_gps_publish_rate_arg,
        gps_navigation_odom_topic_arg,
        enable_gps_navigation_arg,
        prefer_system_python,
        prefer_ros_libs,
        cyclone_participant_limit,
        gazebo_resources,
        # mock_gps_fix_publisher,  # Gazebo GPS is remapped to /gps/fix.
        gps_global_ekf,
        navsat_transform,
        gazebo_launch,
        clock_bridge,
        gps_fix_remap,
        gps_covariance,
        robot_spawn_launch,
        nav2_delayed,
        rviz_delayed,
    ])
