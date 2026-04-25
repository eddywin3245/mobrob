from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg = get_package_share_directory('p3at_robot')

    # Set up LIDAR ethernet interface
    lidar_eth = ExecuteProcess(
        cmd=['bash', '-c',
             'ip addr add 192.168.198.1/24 dev enp89s0 || true && ip link set enp89s0 up'],
        output='log',
    )

    # LIDAR driver
    lidar = Node(
        package='lakibeam1',
        executable='lakibeam1_scan_node',
        name='richbeam_lidar_node0',
        output='log',
        arguments=['--ros-args', '--log-level', 'richbeam_lidar_node0:=WARN'],
        parameters=[{
            'frame_id':         'laser',
            'output_topic':     'scan',
            'inverted':         False,
            'hostip':           '0.0.0.0',
            'port':             '2368',
            'angle_offset':     0,
            'sensorip':         '192.168.198.2',
            'scanfreq':         '30',
            'filter':           '3',
            'laser_enable':     'true',
            'scan_range_start': '45',
            'scan_range_stop':  '315',
        }]
    )

    # LIDAR position on robot: 0.2m forward, 0.3m up, centered
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf',
        arguments=['0.2', '0', '0.3', '0', '0', '0', 'base_link', 'laser']
    )

    # Robot base — publishes /odom and odom->base_link TF
    aria_node = Node(
        package='p3at_robot',
        executable='ariaNode',
        name='aria_node',
        arguments=['-rp', '/dev/ttyUSB0'],
        output='screen'
    )

    # Gamepad — so you can drive around while mapping
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='log'
    )

    gamepad_controller = Node(
        package='p3at_robot',
        executable='gamepad_controller.py',
        name='gamepad_controller',
        output='screen'
    )

    # SLAM toolbox — builds the map from /scan + odom->base_link TF
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg, 'config', 'slam_params.yaml'),
            {'use_sim_time': False}
        ]
    )

    slam_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': ['slam_toolbox']
        }]
    )

    # Foxglove — connect from your laptop browser at ws://<robot-ip>:8765
    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='log',
        parameters=[{'port': 8765}]
    )

    return LaunchDescription([
        lidar_eth,
        lidar,
        laser_tf,
        aria_node,
        joy_node,
        gamepad_controller,
        slam,
        slam_lifecycle,
        foxglove,
    ])
