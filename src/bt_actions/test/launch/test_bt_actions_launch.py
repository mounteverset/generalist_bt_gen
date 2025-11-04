from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
  return LaunchDescription(
    [
      LogInfo(
        msg=(
          "Manual verification steps:\n"
          "  1. In another terminal, run `ros2 action list`.\n"
          "  2. Confirm that only expected action servers for the running system appear; "
          "the bt_actions plugin library does not create action servers on its own.\n"
          "  3. Optionally launch a BehaviorTree.CPP runtime (e.g. bt_executor) that loads "
          "these plugins and observe the action client usage."
        )
      )
    ]
  )
