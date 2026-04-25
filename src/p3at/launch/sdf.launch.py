import os
os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_p3at = get_package_share_directory('p3at')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    gz_resource_path = os.path.dirname(pkg_p3at)
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + gz_resource_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = gz_resource_path

    nav2_params = os.path.join(pkg_p3at, 'nav2_params.yaml')
    slam_params = os.path.join(pkg_p3at, 'slam_params.yaml')
    ekf_params = os.path.join(pkg_p3at, 'ekf.yaml')

    with open(os.path.join(pkg_p3at, 'robots', 'pioneer.urdf'), 'r') as f:
        robot_desc = f.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': os.path.join(pkg_p3at, 'worlds', 'basic_urdf.sdf')}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    # Connect Gazebo's urdf_model/base_link to base_link
    # so robot_state_publisher tree joins Gazebo's TF tree
    base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_tf',
        arguments=['0', '0', '0', '0', '0', '0',
                   'urdf_model/base_link', 'base_link'],
    )

    spawn_robot = ExecuteProcess(
        cmd=["gz", "service", "-s", "/world/pioneer_world/create",
             "--reqtype", "gz.msgs.EntityFactory",
             "--reptype", "gz.msgs.Boolean",
             "--timeout", "1000",
             "--req", 'sdf_filename: "/home/eddywin32/ros2_ws/src/p3at/robots/pioneer.urdf", name: "urdf_model", pose: {position: {z: 0.1}}'],
        output="both"
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': True}]
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                        'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_params,
        }.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params,
            'use_collision_monitor': 'False',
        }.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': True}]
    )
    scan_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='scan_frame_tf',
        arguments=['0', '0', '0', '0', '0', '0',
                   'laser_frame', 'urdf_model/base_link/laser'],
    )
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        bridge,
        base_link_tf,
        TimerAction(period=5.0, actions=[spawn_robot]),
        TimerAction(period=9.0, actions=[ekf]),
        TimerAction(period=15.0, actions=[slam]),
        TimerAction(period=15.0, actions=[nav2]),
        TimerAction(period=17.0, actions=[rviz]),
        scan_frame_tf,
    ])