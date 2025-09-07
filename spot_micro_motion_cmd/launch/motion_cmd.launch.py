from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='spot_micro_motion_cmd',
            executable='spot_micro_motion_cmd_node',
            name='spot_micro_motion_cmd_node',
            output='screen',
            parameters=['config/spot_micro_motion_cmd.yaml']
        ),
    ])
