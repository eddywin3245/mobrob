from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # OAK camera driver — uses DepthAI SDK, publishes to /camera/image_raw
    oak_cam_node = Node(
        package='p3at_robot',
        executable='oak_camera.py',
        name='oak_camera',
        output='screen'
    )

    # Orange cone detection node
    camera_node = Node(
        package='p3at_robot',
        executable='waypoint_camera.py',
        name='camera_node',
        output='screen'
    )

    return LaunchDescription([
        oak_cam_node,
        camera_node,
    ])
