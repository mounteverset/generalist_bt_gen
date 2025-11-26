from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    params_file = Path(__file__).parent.parent / 'config' / 'context_gatherer_params.yaml'
    return LaunchDescription(
        [
            Node(
                package='context_gatherer',
                executable='context_gatherer_node',
                name='context_gatherer',
                output='screen',
                parameters=[str(params_file)],
            )
        ]
    )
