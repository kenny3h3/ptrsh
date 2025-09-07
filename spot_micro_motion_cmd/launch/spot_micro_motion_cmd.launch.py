from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file_motion = os.path.join(
        get_package_share_directory('spot_micro_motion_cmd'),
        'config', 'spot_micro_motion_cmd.yaml'
    )
    config_file_i2c = os.path.join(
        get_package_share_directory('ros2_i2cpwmboard'),
        'config', 'i2cpwm_board.yaml'
    )

    return LaunchDescription([
        Node(
            package="spot_micro_motion_cmd",
            executable="spot_micro_motion_cmd_node",
            name="spot_micro_motion_cmd",
            output="screen",
            parameters=[config_file_motion]
        ),
        Node(
            package="ros2_i2cpwmboard",
            executable="i2cpwm_controller",
            name="i2cpwm_controller",
            output="screen",
            parameters=[config_file_i2c]
        ),
        Node(
            package='spot_micro_keyboard_cmd',
            executable='spot_micro_keyboard_move',
            name='spot_micro_keyboard_cmd',
            output='screen',
        ),
    ])
