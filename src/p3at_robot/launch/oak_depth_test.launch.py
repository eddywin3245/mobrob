#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    oak_depth = Node(
        package='p3at_robot',
        executable='test_oak_depth.py',
        name='oak_depth_test',
        output='screen'
    )

    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='log',
        parameters=[{'port': 8765}]
    )

    return LaunchDescription([
        oak_depth,
        foxglove,
    ])
