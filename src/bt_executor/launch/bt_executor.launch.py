import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('bt_executor')

    default_config = os.path.join(package_share, 'config', 'bt_config.yaml')
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to the TreeExecutionServer parameter YAML file',
    )

    node = Node(
        package='bt_executor',
        executable='bt_executor_node',
        name='bt_action_server',
        output='screen',
        parameters=[LaunchConfiguration('config')],
    )

    return LaunchDescription([config_arg, node])
