from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    sm_pkg = get_package_share_directory('spot_micro_motion_cmd')
    i2c_pkg = get_package_share_directory('i2c_pwm_board')
    kb_pkg = get_package_share_directory('spot_micro_keyboard_cmd')

    sm_cfg  = os.path.join(sm_pkg, 'config', 'spot_micro_motion_cmd.yaml')
    i2c_cfg = os.path.join(i2c_pkg, 'config', 'board.yaml')
    kb_cfg  = os.path.join(kb_pkg, 'config', 'keyboard.yaml') if os.path.exists(os.path.join(kb_pkg, 'config', 'keyboard.yaml')) else None

    nodes = []

    # 1) Treiber (PCA9685)
    nodes.append(Node(
        package='i2c_pwm_board',
        executable='node',
        name='i2c_pwm_board',
        output='screen',
        parameters=[i2c_cfg] if os.path.exists(i2c_cfg) else []
    ))

    # 2) Motion Command Node
    nodes.append(Node(
        package='spot_micro_motion_cmd',
        executable='spot_micro_motion_cmd_node',
        name='spot_micro_motion_cmd',
        output='screen',
        parameters=[sm_cfg] if os.path.exists(sm_cfg) else []
        # remappings=[('servo_absolute', '/servo_absolute')]  # bei Bedarf
    ))

    # 3) Keyboard Teleop
    nodes.append(Node(
        package='spot_micro_keyboard_cmd',
        executable='spot_micro_keyboard_cmd',
        name='spot_micro_keyboard_cmd',
        output='screen',
        parameters=[kb_cfg] if kb_cfg else []
        # remappings=[('cmd', '/cmd')]  # anpassen, falls dein Node andere Topicnamen erwartet
    ))

    return LaunchDescription(nodes)
