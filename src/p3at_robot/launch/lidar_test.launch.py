from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg = 'p3at_robot'

    # Set up LIDAR Ethernet interface
    lidar_eth = ExecuteProcess(
        cmd=['bash', '-c',
             'ip addr add 192.168.198.1/24 dev enp89s0 || true && ip link set enp89s0 up'],
        output='log',
    )

    # Minimal stack: LIDAR only
    lidar = Node(
        package='lakibeam1',
        executable='lakibeam1_scan_node',
        name='richbeam_lidar_node0',
        output='log',
        arguments=['--ros-args', '--log-level', 'richbeam_lidar_node0:=INFO'],
        parameters=[{
            'frame_id': 'laser',
            'output_topic': 'scan',
            'inverted': False,
            'hostip': '0.0.0.0',
            'port': '2368',
            'angle_offset': 0,
            'sensorip': '192.168.198.2',
            'scanfreq': '30',
            'filter': '3',
            'laser_enable': 'true',
            'scan_range_start': '45',
            'scan_range_stop': '315',
        }]
    )

    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf',
        arguments=['0.2', '0', '0.3', '0', '0', '0', 'base_link', 'laser']
    )

    return LaunchDescription([
        lidar_eth,
        lidar,
        laser_tf,
    ])
