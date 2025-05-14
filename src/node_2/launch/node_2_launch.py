from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_node = os.path.join(
        get_package_share_directory('node_2'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='node_2',
            executable='node_2_exe',
            name='node_2',
            output='screen',
            parameters=[config_node],
        )
    ])