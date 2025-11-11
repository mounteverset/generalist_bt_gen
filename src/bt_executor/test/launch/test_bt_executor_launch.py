import os
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.tools
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


@pytest.mark.launch_test
def generate_test_description():
    package_share = get_package_share_directory('bt_executor')
    config_path = os.path.join(package_share, 'config', 'bt_config.yaml')

    bt_node = Node(
        package='bt_executor',
        executable='bt_executor_node',
        name='bt_action_server',
        output='screen',
        parameters=[config_path],
    )

    ld = launch.LaunchDescription([bt_node, launch_testing.actions.ReadyToTest()])

    return ld, {'bt_process': bt_node}


class TestBtExecutorLaunch(unittest.TestCase):

    def test_process_starts(self, bt_process, proc_info):
        proc_info.assertWaitForStartup(process=bt_process, timeout=10.0)

    def test_node_visible(self, bt_process):
        rclpy.init()
        node = rclpy.create_node('bt_executor_launch_test')

        try:
            def is_present():
                return '/bt_action_server' in node.get_node_names()

            self.assertTrue(launch_testing.tools.wait_for(is_present, timeout=5.0))
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class BtExecutorPostShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info, bt_process):
        proc_info.assertWaitForShutdown(process=bt_process, timeout=5.0)
        proc_info.assertExitCodes(process=bt_process)
