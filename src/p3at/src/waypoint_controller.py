#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class WaypointController(Node):
    def __init__(self):
        super().__init__('waypoint_controller')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # Obstacle avoidance
        self.obstacle_detected = False
        self.obstacle_left = False
        self.obstacle_right = False
        self.min_obstacle_dist = 1  
        
        # Waypoints (x, y, heading_degrees)
        self.waypoints = [
            (2.0, 0.0, 0.0),
            (2.0, 2.0, 90.0),
            (0.0, 2.0, 180.0),
            (0.0, 0.0, 0.0),
        ]
        
        self.current_waypoint = 0
        self.state = 'rotating'
        
        self.linear_kp = 0.5
        self.angular_kp = 1.0
        self.dist_tolerance = 0.2
        self.angle_tolerance = 0.05
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Waypoint Controller with obstacle avoidance started!')

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def scan_callback(self, msg):
        ranges = msg.ranges
        num_ranges = len(ranges)
        
        # Filter out inf and nan values
        valid = lambda r: not math.isinf(r) and not math.isnan(r) and r > 0.1
        
        # Front sector - check for obstacles ahead
        front_size = num_ranges // 6  
        front_ranges = [ranges[i] for i in range(front_size) if valid(ranges[i])]
        front_ranges += [ranges[-(i+1)] for i in range(front_size) if valid(ranges[-(i+1)])]
        
        # Left and right sectors
        left_size = num_ranges // 4
        left_ranges = [ranges[num_ranges//4 + i] for i in range(left_size) if valid(ranges[num_ranges//4 + i])]
        right_ranges = [ranges[i] for i in range(left_size) if valid(ranges[i])]
        
        min_front = min(front_ranges) if front_ranges else float('inf')
        min_left = min(left_ranges) if left_ranges else float('inf')
        min_right = min(right_ranges) if right_ranges else float('inf')
        
        self.obstacle_detected = min_front < self.min_obstacle_dist
        self.obstacle_left = min_left < self.min_obstacle_dist
        self.obstacle_right = min_right < self.min_obstacle_dist

    def control_loop(self):
        if self.state == 'done' or self.current_waypoint >= len(self.waypoints):
            self.stop_robot()
            if self.state != 'done':
                self.state = 'done'
                self.get_logger().info('All waypoints reached!')
            return

        if self.obstacle_detected and self.state == 'driving':
            self.get_logger().info('Obstacle detected! Avoiding...')
            twist = Twist()
            twist.linear.x = 0.0
            if not self.obstacle_right:
                twist.angular.z = -0.5  # turn right
            else:
                twist.angular.z = 0.5   # turn left
            self.cmd_vel_pub.publish(twist)
            return

        target_x, target_y, target_heading = self.waypoints[self.current_waypoint]
        
        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_goal - self.yaw)

        if self.state == 'rotating':
            if abs(angle_error) > self.angle_tolerance:
                twist = Twist()
                twist.angular.z = self.angular_kp * angle_error
                twist.angular.z = max(-1.0, min(1.0, twist.angular.z))
                self.cmd_vel_pub.publish(twist)
            else:
                self.state = 'driving'
                self.get_logger().info(f'Driving to waypoint {self.current_waypoint + 1}')

        elif self.state == 'driving':
            if distance > self.dist_tolerance:
                twist = Twist()
                twist.linear.x = self.linear_kp * distance
                twist.linear.x = max(0.0, min(0.5, twist.linear.x))
                twist.angular.z = self.angular_kp * angle_error
                twist.angular.z = max(-0.5, min(0.5, twist.angular.z))
                self.cmd_vel_pub.publish(twist)
            else:
                self.state = 'aligning'
                self.get_logger().info(f'Reached waypoint {self.current_waypoint + 1}, aligning...')

        elif self.state == 'aligning':
            target_yaw = math.radians(target_heading)
            heading_error = self.normalize_angle(target_yaw - self.yaw)
            
            if abs(heading_error) > self.angle_tolerance:
                twist = Twist()
                twist.angular.z = self.angular_kp * heading_error
                twist.angular.z = max(-0.5, min(0.5, twist.angular.z))
                self.cmd_vel_pub.publish(twist)
            else:
                self.stop_robot()
                self.get_logger().info(f'Waypoint {self.current_waypoint + 1} complete!')
                self.current_waypoint += 1
                if self.current_waypoint < len(self.waypoints):
                    self.state = 'rotating'
                else:
                    self.state = 'done'

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    controller = WaypointController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()