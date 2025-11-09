import json
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from launch_ros.actions import Node
from std_srvs.srv import Trigger

import unittest

try:
    import pytest
except ModuleNotFoundError:  # pragma: no cover
    class _PytestStub:
        class mark:
            @staticmethod
            def launch_test(func=None, **_unused_kwargs):
                if func is None:
                    def decorator(inner):
                        return inner
                    return decorator
                return func

        @staticmethod
        def skip(reason):
            raise unittest.SkipTest(reason)

    pytest = _PytestStub()

try:
    import rclpy
    _RCLPY_IMPORT_ERROR = None
except Exception as exc:  # pragma: no cover
    rclpy = None
    _RCLPY_IMPORT_ERROR = exc


@pytest.mark.launch_test
def generate_test_description():
    if rclpy is None:
        pytest.skip(f"rclpy is unavailable: {_RCLPY_IMPORT_ERROR}")

    context_node = Node(
        package='context_gatherer',
        executable='context_gatherer_node',
        name='context_gatherer',
        output='screen',
    )

    mock_publishers = Node(
        package='context_gatherer',
        executable='context_gatherer_mock_sensor_node',
        name='mock_sensor_publisher',
        output='screen',
    )

    ld = launch.LaunchDescription([
        context_node,
        mock_publishers,
        launch_testing.actions.ReadyToTest(),
    ])

    return ld, {
        'context_process': context_node,
        'mock_process': mock_publishers,
    }


class TestContextGathererLaunch(unittest.TestCase):

    @staticmethod
    def _wait_for(predicate, timeout_sec, poll_period=0.1):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if predicate():
                return True
            time.sleep(poll_period)
        return False

    def test_nodes_start(self, proc_info, context_process, mock_process):
        proc_info.assertWaitForStartup(process=context_process, timeout=10.0)
        proc_info.assertWaitForStartup(process=mock_process, timeout=10.0)

    def test_context_service_response(self, context_process):
        if rclpy is None:  # pragma: no cover
            self.skipTest(f"rclpy is unavailable: {_RCLPY_IMPORT_ERROR}")
        try:
            rclpy.init()
        except Exception as exc:  # pragma: no cover
            self.skipTest(f"Failed to initialize rclpy: {exc}")
        node = rclpy.create_node('context_gatherer_launch_test')

        try:
            client = node.create_client(Trigger, 'get_context')

            def service_ready():
                return client.wait_for_service(timeout_sec=0.1)

            self.assertTrue(
                self._wait_for(service_ready, timeout_sec=5.0),
                msg='get_context service not available',
            )

            response = None
            for _ in range(5):
                future = client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
                if future.done():
                    response = future.result()
                    if response and response.success:
                        break
                time.sleep(0.5)

            self.assertIsNotNone(response, msg='No response received from get_context service')
            self.assertTrue(response.success, msg=f'Context gatherer reported failure: {response.message}')

            payload = json.loads(response.message)
            self.assertIn('camera', payload)
            self.assertIn('odometry', payload)
            self.assertIn('map', payload)
            self.assertIn('geolocation', payload)
            metadata = payload.get('metadata', {})
            sources = set(metadata.get('available_sources', []))
            self.assertTrue({'camera', 'odometry', 'map', 'geolocation'}.issubset(sources))
        finally:
            node.destroy_client(client)
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class ContextGathererPostShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info, context_process, mock_process):
        launch_testing.asserts.assertExitCodes(proc_info, process=context_process)
        launch_testing.asserts.assertExitCodes(proc_info, process=mock_process)
