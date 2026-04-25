from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    teleop_config = os.path.join(
        get_package_share_directory('p3at'), 'config', 'teleop.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    gamepad_controller = Node(
        package='p3at',
        executable='gamepad_controller.py',
        name='gamepad_controller',
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        gamepad_controller,
    ])
