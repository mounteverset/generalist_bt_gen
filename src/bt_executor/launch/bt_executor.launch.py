from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('bt_executor'),
        'config',
        'bt_executor_params.yaml',
    )

    return LaunchDescription(
        [
            Node(
                package='bt_executor',
                executable='bt_executor_node',
                name='bt_action_server',
                output='screen',
                parameters=[params_file],
            )
        ]
    )
