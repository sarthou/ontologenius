import os
import unittest
import launch
import launch_ros
import launch_testing
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_test_description():
    ontologenius_launch = os.path.join(
        get_package_share_directory('ontologenius'),
        'launch',
        'ontologenius.py'
    )

    test_node = launch_ros.actions.Node(
        package='ontologenius',
        executable='onto_api_relations_test',
        name='api_relations_test',
        output='screen'
    )

    return LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(ontologenius_launch),
            launch_arguments={'intern_file': 'none', 'display': 'false'}.items(),
        ),

        test_node,
        launch_testing.actions.ReadyToTest(),
    ]), {"test_node": test_node}

# Optional: assert the test process exited cleanly
import unittest

# ================= ACTIVE PHASE =================
class TestWhileRunning(unittest.TestCase):
    def test_gtest_runs_to_completion(self, proc_info, test_node):
        """Wait for the gtest process to exit (blocks until it does)."""
        proc_info.assertWaitForShutdown(process=test_node, timeout=60)

@launch_testing.post_shutdown_test()
class TestAfterShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
