from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
  return LaunchDescription(
    [
      LogInfo(
        msg=(
          "Manual verification steps:\n"
          "  1. Start the LLM interface service node (once implemented).\n"
          "  2. In another terminal, run `ros2 service list` and confirm `/llm_query` "
          "appears with type `llm_interface/srv/LLMQuery`.\n"
          "  3. Optionally load a behavior tree that uses ThinkingNode or "
          "BlackboardUpdateNode to observe live interactions."
        )
      )
    ]
  )
