from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_i2cpwmboard')
    cfg = os.path.join(pkg_share, 'config', 'i2cpwm_board.yaml')

    return LaunchDescription([
        Node(
            package='ros2_i2cpwmboard',
            executable='i2cpwm_controller',
            name='i2cpwm_controller',
            output='screen',
            parameters=[cfg] if os.path.exists(cfg) else []
        )
    ])
