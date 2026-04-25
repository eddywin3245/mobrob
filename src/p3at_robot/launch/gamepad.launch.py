from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os
from datetime import datetime

# LIDAR position on robot: x=0.2m forward, z=0.3m height, centered

def generate_launch_description():

    pkg = get_package_share_directory('p3at_robot')

    # Set up the LIDAR Ethernet interface (works because container runs --privileged).
    # '|| true' ensures a non-zero exit (address already assigned) does not crash the launch.
    lidar_eth = ExecuteProcess(
        cmd=['bash', '-c',
             'ip addr add 192.168.198.1/24 dev enp89s0 || true && ip link set enp89s0 up'],
        output='log',
    )

    # Record all ROS topics to a rosbag (timestamped directory)
    bag_dir = f'/root/bags/run_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
    rosbag_recorder = ExecuteProcess(
        cmd=['bash', '-c', f'mkdir -p /root/bags && ros2 bag record -a -o {bag_dir}'],
        output='log',
    )

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

    aria_node = Node(
        package='p3at_robot',
        executable='ariaNode',
        name='aria_node',
        arguments=['-rp', '/dev/ttyUSB0'],
        output='screen'
    )

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

    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf',
        arguments=['0.2', '0', '0.3', '0', '0', '0', 'base_link', 'laser']
    )

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

    nav2_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
            ]
        }]
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'nav2_params.yaml')],
        remappings=[('cmd_vel', 'nav2/cmd_vel')]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'nav2_params.yaml')]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'nav2_params.yaml')],
        remappings=[('cmd_vel', 'nav2/cmd_vel')]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'nav2_params.yaml')]
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'nav2_params.yaml')]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'ekf.yaml')]
    )

    phidget_imu = Node(
        package='p3at_robot',
        executable='phidget_imu.py',
        name='phidget_imu',
        output='log'
    )

    gps_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='gps_node',
        output='screen',
        parameters=[{'port': '/dev/ttyACM0', 'baud': 9600}]
    )

    gps_navigator = Node(
        package='p3at_robot',
        executable='gps_waypoint_navigator.py',
        name='gps_waypoint_navigator',
        output='screen'
    )

    oak_camera = Node(
        package='p3at_robot',
        executable='oak_camera.py',
        name='oak_camera',
        output='screen'
    )

    object_detector = Node(
        package='p3at_robot',
        executable='object_detector.py',
        name='object_detector',
        output='screen'
    )

    camera_rgb_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_rgb_tf',
        arguments=['0.15', '0', '0.1', '0', '0', '0', 'base_link', 'camera_rgb_optical_frame']
    )

    camera_depth_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_depth_tf',
        arguments=['0.15', '0', '0.1', '0', '0', '0', 'base_link', 'camera_depth_optical_frame']
    )

    cone_detection = Node(
        package='p3at_robot',
        executable='cone_detection.py',
        name='cone_detection',
        output='screen'
    )

    waypoint_scanner = Node(
        package='p3at_robot',
        executable='waypoint_scanner.py',
        name='waypoint_scanner',
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
        lidar_eth,
        rosbag_recorder,
        joy_node,
        gamepad_controller,
        aria_node,
        lidar,
        laser_tf,
        ekf_node,
        slam,
        slam_lifecycle,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        nav2_lifecycle,
        phidget_imu,
        # gps_node,       # not used in Part 3 (indoor/confined area, LIDAR SLAM used instead)
        # gps_navigator,  # not used in Part 3
        oak_camera,
        object_detector,
        camera_rgb_tf,
        camera_depth_tf,
        cone_detection,
        waypoint_scanner,
        foxglove,
    ])
