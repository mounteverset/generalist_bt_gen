#!/usr/bin/env python3
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, NotSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    #t_executor_share = Path(get_package_share_directory('bt_executor'))
    bt_executor_params = Path(get_package_share_directory('bt_executor')) / 'config' / 'bt_executor_params.yaml'
    mission_params = Path(get_package_share_directory('mission_coordinator')) / 'config' / 'mission_coordinator_params.yaml'
    ui_params = Path(get_package_share_directory('user_interface')) / 'config' / 'chat_params.yaml'
    llm_params = Path(get_package_share_directory('llm_interface')) / 'config' / 'llm_interface_params.yaml'
    context_params = Path(get_package_share_directory('context_gatherer')) / 'config' / 'context_gatherer_params.yaml'

    params_file_arg = DeclareLaunchArgument(
        'bt_executor_params',
        default_value=str(bt_executor_params),
        description='Path to the bt_executor parameter file.'
    )
    mission_params_arg = DeclareLaunchArgument(
        'mission_coordinator_params',
        default_value=str(mission_params),
        description='Path to the mission_coordinator parameter file.'
    )
    ui_params_arg = DeclareLaunchArgument(
        'user_interface_params',
        default_value=str(ui_params),
        description='Path to the user_interface chat parameter file.'
    )
    llm_params_arg = DeclareLaunchArgument(
        'llm_interface_params',
        default_value=str(llm_params),
        description='Path to the llm_interface parameter file.'
    )
    context_params_arg = DeclareLaunchArgument(
        'context_gatherer_params',
        default_value=str(context_params),
        description='Path to the context_gatherer parameter file.'
    )
    use_cli_arg = DeclareLaunchArgument(
        'use_cli_ui',
        default_value='false',
        description='Launch CLI chat node if true, otherwise launch the web UI node.'
    )

    bt_executor_node = Node(
        package='bt_executor',
        executable='bt_executor_node',
        #name='bt_action_server',
        output='screen',
        parameters=[LaunchConfiguration('bt_executor_params')]
    )

    llm_interface_node = Node(
        package='llm_interface',
        executable='llm_interface_node',
        name='llm_interface',
        output='screen',
        parameters=[LaunchConfiguration('llm_interface_params')]
    )

    context_gatherer_node = Node(
        package='context_gatherer',
        executable='context_gatherer_node',
        name='context_gatherer',
        output='screen',
        parameters=[LaunchConfiguration('context_gatherer_params')],
        remappings=[
            ('/camera/image_raw', '/a200_0000/sensors/camera_0/color/image'),
            ('/camera/depth/image_raw', '/a200_0000/sensors/camera_0/depth/image'),
            ('/map', '/a200_0000/map'),
        ],
    )

    mission_coordinator_node = Node(
        package='mission_coordinator',
        executable='mission_coordinator_node',
        name='mission_coordinator',
        output='screen',
        parameters=[LaunchConfiguration('mission_coordinator_params')]
    )

    cli_chat_node = Node(
        condition=IfCondition(LaunchConfiguration('use_cli_ui')),
        package='user_interface',
        executable='chat_node',
        name='chat_ui_cli',
        output='screen',
        parameters=[LaunchConfiguration('user_interface_params')]
    )

    web_ui_node = Node(
        condition=IfCondition(NotSubstitution(LaunchConfiguration('use_cli_ui'))),
        package='user_interface',
        executable='web_ui_node',
        name='chat_ui_web',
        output='screen',
        parameters=[LaunchConfiguration('user_interface_params')]
    )

    dummy_log_temperature_node = Node(
        package='generalist_bringup',
        executable='dummy_log_temperature_node',
        name='dummy_log_temperature',
        output='screen',
    )

    return LaunchDescription([
        params_file_arg,
        mission_params_arg,
        ui_params_arg,
        llm_params_arg,
        context_params_arg,
        use_cli_arg,
        bt_executor_node,
        llm_interface_node,
        context_gatherer_node,
        mission_coordinator_node,
        cli_chat_node,
        web_ui_node,
        dummy_log_temperature_node,
    ])
