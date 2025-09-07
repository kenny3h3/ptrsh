from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='spot_micro_keyboard_cmd',
            executable='spot_micro_keyboard_move',
            name='spot_micro_keyboard_cmd',
            output='screen',
        ),
    ])
