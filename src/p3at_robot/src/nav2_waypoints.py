#!/usr/bin/env python3
"""
Autonomous navigation using Nav2.
Usage:
  ros2 run p3at_robot nav2_waypoints.py          <- drives 2x2m square
  ros2 run p3at_robot nav2_waypoints.py spin     <- spins 90 degrees (quick test)
"""
import sys
import math
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


def get_current_pose():
    """Get robot's current map-frame pose via TF2 (map -> base_link)."""
    import tf2_ros
    from rclpy.node import Node

    node = Node('_pose_getter_tmp')
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node)

    # Spin until the map->base_link transform is available (up to 15 s)
    for _ in range(150):
        try:
            t = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            node.destroy_node()
            x   = t.transform.translation.x
            y   = t.transform.translation.y
            q   = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return x, y, yaw, 'map'
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    raise RuntimeError('Could not get map->base_link TF. Is SLAM running?')


def make_pose(nav, robot_x, robot_y, robot_yaw, frame, local_x, local_y, local_yaw_deg):
    """Convert a robot-relative offset into a PoseStamped in the given frame."""
    map_x   = robot_x + local_x * math.cos(robot_yaw) - local_y * math.sin(robot_yaw)
    map_y   = robot_y + local_x * math.sin(robot_yaw) + local_y * math.cos(robot_yaw)
    map_yaw = robot_yaw + math.radians(local_yaw_deg)

    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = map_x
    pose.pose.position.y = map_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(map_yaw / 2.0)
    pose.pose.orientation.w = math.cos(map_yaw / 2.0)
    return pose


def main():
    rclpy.init()
    nav = BasicNavigator()

    mode = sys.argv[1] if len(sys.argv) > 1 else 'waypoints'

    print('Waiting for Nav2 to become active...')
    nav._waitForNodeToActivate('bt_navigator')
    print('Nav2 is active!')

    if mode == 'spin':
        # --- Quick test: spin 90 degrees in place ---
        print('Spinning 90 degrees...')
        nav.spin(spin_dist=math.pi / 2)   # 1.5708 rad = 90 deg
        while not nav.isTaskComplete():
            pass
        result = nav.getResult()
        print('Spin result:', result)

    else:
        # --- Full waypoint square ---
        print('Getting current robot pose...')
        robot_x, robot_y, robot_yaw, frame = get_current_pose()
        print(f'Robot at ({robot_x:.2f}, {robot_y:.2f}), '
              f'heading {math.degrees(robot_yaw):.1f} deg  [{frame} frame]')

        # (forward m, left m, heading offset deg) relative to current pose
        relative_waypoints = [
            (2.0, 0.0,   0.0),
            (2.0, 2.0,  90.0),
            (0.0, 2.0, 180.0),
            (0.0, 0.0,   0.0),
        ]

        waypoints = [
            make_pose(nav, robot_x, robot_y, robot_yaw, frame, lx, ly, lyaw)
            for lx, ly, lyaw in relative_waypoints
        ]

        print(f'Sending {len(waypoints)} waypoints...')
        nav.followWaypoints(waypoints)

        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback:
                print(f'Waypoint {feedback.current_waypoint + 1}/{len(waypoints)}')

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print('All waypoints reached!')
        elif result == TaskResult.CANCELED:
            print('Navigation canceled.')
        elif result == TaskResult.FAILED:
            print('Navigation failed.')

    nav.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()