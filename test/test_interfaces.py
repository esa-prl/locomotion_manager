import time
import pytest
import unittest
import rclpy
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing

from rover_msgs.srv import ChangeLocomotionMode

from launch_helpers import get_ws_src_directory, add_namespace_to_yaml, to_urdf

namespace_ = 'marta'

# Get package src path based on a package name. Make sure the package is installed from source.
ros2_ws_src = get_ws_src_directory('locomotion_manager')

# This is necessary to get unbuffered output from the process under test
proc_env = os.environ.copy()
proc_env['PYTHONUNBUFFERED'] = '1'


@pytest.mark.launch_test
def generate_test_description():

    # Load XACRO and parse to URDF
    pkg_rover_config = get_package_share_directory('rover_config')
    xacro_model_name = "marta.xacro"
    xacro_model_path = os.path.join(pkg_rover_config, 'urdf', xacro_model_name)
    # Parse XACRO file to URDF
    urdf_model_path = to_urdf(xacro_model_path)
    urdf_params = {'urdf_model_path': urdf_model_path}

    stop_mode_config = os.path.join(ros2_ws_src, 'locomotion_mode', 'config', 'stop_mode.yaml')
    locomotion_manager_config = os.path.join(ros2_ws_src, 'locomotion_manager', 'config',
                                             'test_config.yaml')
    locomotion_manager_config_ns = add_namespace_to_yaml(namespace_, locomotion_manager_config)
    stop_mode_config_ns = add_namespace_to_yaml(namespace_, stop_mode_config)

    locomotion_manager_node = Node(
        package='locomotion_manager',
        node_namespace=namespace_,
        node_executable='locomotion_manager_node',
        node_name='locomotion_manager_node',
        output='screen',
        parameters=[locomotion_manager_config_ns],
        emulate_tty=True,
        env=proc_env,
    )

    stop_mode_node = Node(
        package='locomotion_mode',
        node_namespace=namespace_,
        node_executable='stop_mode_node',
        node_name='stop_mode_node',
        output='screen',
        parameters=[urdf_params, stop_mode_config_ns],
        emulate_tty=True,
        env=proc_env,
    )

    return (LaunchDescription([
        locomotion_manager_node,
        stop_mode_node,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'locomotion_manager_proc': locomotion_manager_node,
        'stop_mode_node_proc': stop_mode_node,
    })


class TestInterfaces(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create ROS node for input
        self.input_node = rclpy.create_node('test_input_node', namespace=namespace_)

    def tearDown(self):
        self.input_node.destroy_node()

    def test_change_to_stop_mode(self, locomotion_manager_proc, stop_mode_node_proc, proc_output,
                                 proc_info):
        '''
        This test requests a change to the stop_mode from the locomotion_manager.
        The stop_mode is expected to be properly setup and defined.
        '''
        try:
            client = self.input_node.create_client(ChangeLocomotionMode, 'change_locomotion_mode')

            end_time = time.time() + 5
            while time.time() < end_time:
                pass

            # Wait for the service to be available
            tries = 15
            while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0) and tries > 0:
                print('service not available, waiting again...')
                tries -= 1
            assert tries > 0, 'service still not available, aborting test'

            # make a call to the change-locomotion_mode service
            request = ChangeLocomotionMode.Request()
            request.new_locomotion_mode = 'stop_mode_node'

            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.input_node, future, timeout_sec=3)
            result = future.result()

            assert result.success is True

        finally:
            self.input_node.destroy_client(client)

    def test_change_to_unconfigured_mode(self, locomotion_manager_proc, proc_output, proc_info):
        '''
        This test requests a locomotion mode from the locomotion_manager,
        that is neither given in the used config, nor is defined in locomotion_mode
        '''
        try:
            client = self.input_node.create_client(ChangeLocomotionMode, 'change_locomotion_mode')

            end_time = time.time() + 5
            while time.time() < end_time:
                pass

            # Wait for the service to be available
            tries = 15
            while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0) and tries > 0:
                print('service not available, waiting again...')
                tries -= 1
            assert tries > 0, 'service still not available, aborting test'

            # Make a request for a not defned mode
            request = ChangeLocomotionMode.Request()
            request.new_locomotion_mode = 'unconfigured_mode'

            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.input_node, future, timeout_sec=3)
            result = future.result()

            assert result.success is False

        finally:
            self.input_node.destroy_client(client)

    def test_change_to_undefined_mode(self, locomotion_manager_proc, proc_output, proc_info):
        '''
        This test requests a locomotion mode from the locomotion_manager,
        that is given in the config, but not a defined locomotion_mode
        '''
        try:
            client = self.input_node.create_client(ChangeLocomotionMode, 'change_locomotion_mode')

            end_time = time.time() + 5
            while time.time() < end_time:
                pass

            # Wait for the service to be available
            tries = 15
            while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0) and tries > 0:
                print('service not available, waiting again...')
                tries -= 1
            assert tries > 0, 'service still not available, aborting test'

            # Make a request for a not defned mode
            request = ChangeLocomotionMode.Request()
            request.new_locomotion_mode = 'undefined_mode'

            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.input_node, future, timeout_sec=3)
            result = future.result()

            assert result.success is False

        finally:
            self.input_node.destroy_client(client)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self):
        launch_testing.asserts.assertExitCodes(self.proc_info)
