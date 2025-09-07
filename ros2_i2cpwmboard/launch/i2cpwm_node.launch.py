from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('ros2_i2cpwmboard'),
        'config', 'i2cpwm_board.yaml'
    )

    return LaunchDescription([
        Node(
            package="ros2_i2cpwmboard",
            executable="i2cpwm_controller",
            name="i2cpwm_controller",
            output="screen",
            parameters=[config_file]
        )
    ])
