import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_prompt_config = os.path.join(
        get_package_share_directory("llm_interface"), "config", "prompt_config.yaml"
    )

    prompt_config = DeclareLaunchArgument(
        "prompt_config_path",
        default_value=default_prompt_config,
        description="Path to the prompt configuration YAML file.",
    )

    model = DeclareLaunchArgument(
        "model",
        default_value="",
        description="Override the default model defined in the prompt configuration.",
    )

    temperature = DeclareLaunchArgument(
        "temperature",
        default_value="0.2",
        description="Sampling temperature applied to LLM requests.",
    )

    max_tokens = DeclareLaunchArgument(
        "max_tokens",
        default_value="2048",
        description="Maximum number of tokens returned by the LLM.",
    )

    response_format = DeclareLaunchArgument(
        "response_format",
        default_value="json_object",
        description="Expected response format for the LLM.",
    )

    service_name = DeclareLaunchArgument(
        "service_name",
        default_value="llm_query",
        description="Service name exposed by the node.",
    )

    node = Node(
        package="llm_interface",
        executable="llm_service_node",
        output="screen",
        parameters=[{
            "prompt_config_path": LaunchConfiguration("prompt_config_path"),
            "model": LaunchConfiguration("model"),
            "temperature": LaunchConfiguration("temperature"),
            "max_tokens": LaunchConfiguration("max_tokens"),
            "response_format": LaunchConfiguration("response_format"),
            "service_name": LaunchConfiguration("service_name"),
        }],
    )

    return LaunchDescription([
        prompt_config,
        model,
        temperature,
        max_tokens,
        response_format,
        service_name,
        node,
    ])
