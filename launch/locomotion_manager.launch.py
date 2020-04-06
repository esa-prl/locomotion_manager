from launch import LaunchDescription
from launch_ros.actions import Node
from launch_helpers import get_ws_src_directory, add_namespace_to_yaml
import os

namespace_ = 'marta'

# Get package src path based on a package name. Make sure the package is installed from source.
ros2_ws_src = get_ws_src_directory('locomotion_manager')


def generate_launch_description():
    # Individual Parameter files
    locomotion_manager_config = os.path.join(ros2_ws_src, 'locomotion_manager', 'config', 'locomotion_manager.yaml')
    # Add namespace to the yaml file
    locomotion_manager_config_ns=add_namespace_to_yaml(
        namespace_, locomotion_manager_config)
    
    return LaunchDescription([
        Node(
            package='locomotion_manager',
            node_namespace=namespace_,
            node_executable='locomotion_manager_node',
            node_name='locomotion_manager_node',
            output='screen',
            parameters=[locomotion_manager_config_ns],
            emulate_tty=True
        )
    ])
