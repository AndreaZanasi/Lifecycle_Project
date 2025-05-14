from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_node = os.path.join(
        get_package_share_directory('life_manager'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='life_manager',
            executable='life_manager_exe',
            name='life_manager',
            output='screen',
            parameters=[config_node],
        )
    ])