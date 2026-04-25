#!/usr/bin/env python3
"""
Gamepad controller — mode structure:

- Circle (O):  MANUAL. Pure joystick, no deadman. Always takes precedence.
- X:           AUTO. Parent mode; requires R2 trigger (deadman) for motion.
    - Square (in AUTO + deadman): trigger GPS waypoint navigation.
    - Triangle (in AUTO):         switch to ASSIST-drive mode.
- ASSIST:      Stick drives; on LIDAR obstacle in front, auto-executes a
               90° turn + 1.5 m-radius semicircle + 90° return turn so the
               robot ends up past the obstacle facing its original heading,
               then hands back to the stick.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool

# Button indices (PS4 Joy message)
BTN_X = 0
BTN_CIRCLE = 1
BTN_SQUARE = 2
BTN_TRIANGLE = 3

# Axis indices
AXIS_LEFT_LR = 0
AXIS_LEFT_UD = 1
AXIS_L2 = 2
AXIS_R2 = 5

# Assist-mode obstacle avoidance
# Trigger at 1.7 m LIDAR range so that after the 90° turn the cone ends up
# ~1.5 m to the side (LIDAR is mounted 0.2 m forward of base_link), which
# lands exactly at the arc centre → robot stays 1.5 m from cone throughout.
OBSTACLE_TRIGGER_DIST = 0.8    # LIDAR range (m) that kicks off the maneuver
FRONT_CONE_HALF_DEG   = 30.0    # ±30° front cone for trigger check

AVOID_DIR             = -1                      # -1 = go around on the right, +1 = left
TURN_ANGULAR          = 0.5                     # rad/s during initial rotate
TURN_DURATION_S       = math.radians(90) / 0.5  # ≈ 3.14 s for 90°
ARC_LINEAR            = 0.3                     # m/s during semicircle
ARC_RADIUS            = 2                     # m
ARC_ANGULAR           = ARC_LINEAR / ARC_RADIUS # = 0.2 rad/s
ARC_SWEEP_RAD         = math.pi                 # 180° = semicircle
ARC_DURATION_S        = ARC_SWEEP_RAD / ARC_ANGULAR  # ≈ 15.7 s


class GamepadController(Node):
    def __init__(self):
        super().__init__('gamepad_controller')

        self.mode = 'manual'
        self.deadman_pressed = False
        self.controller_connected = False
        self.last_joy_time = None
        self._scan = None

        # Assist-mode state machine: 'drive' | 'turn' | 'arc' | 'return'
        self._assist_state = 'drive'
        self._maneuver_start = 0.0

        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Twist, '/gps_nav/cmd_vel', self._gps_nav_vel_cb, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub    = self.create_publisher(String, '/robot_mode', 10)
        self.gps_nav_pub = self.create_publisher(Bool, '/start_gps_nav', 10)

        self.create_timer(1.0, self._check_controller)
        self.create_timer(0.1, self._enforce_deadman)
        self.create_timer(0.1, self._assist_tick)

        self.get_logger().info('=== Gamepad Controller ===')
        self.get_logger().info('O=MANUAL (no deadman, wins over everything)')
        self.get_logger().info('X=AUTO  (hold R2; Square=GPS, Triangle=assist)')
        self.get_logger().info('Waiting for controller...')

    def _joy_cb(self, msg):
        if not self.controller_connected:
            self.get_logger().info('Controller connected!')
            self.controller_connected = True
        self.last_joy_time = self.get_clock().now()

        r2 = msg.axes[AXIS_R2] if len(msg.axes) > AXIS_R2 else 1.0
        self.deadman_pressed = r2 < 0.0

        # O always wins — snap to manual
        if msg.buttons[BTN_CIRCLE] == 1 and self.mode != 'manual':
            self.mode = 'manual'
            self._assist_state = 'drive'
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info('→ MANUAL mode')
            self._publish_mode()
        # X → auto (unless we just went manual this tick)
        elif msg.buttons[BTN_X] == 1 and self.mode != 'auto':
            self.mode = 'auto'
            self._assist_state = 'drive'
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info('→ AUTO mode — hold R2 to move')
            self._publish_mode()

        if self.mode == 'manual':
            self._drive_manual(msg)
        elif self.mode == 'auto':
            self._auto_mode(msg)
        elif self.mode == 'assist':
            self._drive_assist(msg)

    def _drive_manual(self, msg):
        twist = Twist()
        twist.linear.x  = msg.axes[AXIS_LEFT_UD] * 1.0
        twist.angular.z = msg.axes[AXIS_LEFT_LR] * 1.0
        self.cmd_vel_pub.publish(twist)

    def _auto_mode(self, msg):
        # Triangle switches to assist — no deadman needed (just a mode change)
        if msg.buttons[BTN_TRIANGLE] == 1:
            self.mode = 'assist'
            self._assist_state = 'drive'
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info('→ ASSIST mode — stick drives; auto-orbits obstacles')
            self._publish_mode()
            return

        # Square triggers GPS nav — needs deadman since it launches autonomous motion
        if self.deadman_pressed and msg.buttons[BTN_SQUARE] == 1:
            self.get_logger().info('Triggering GPS waypoint navigation...')
            self.gps_nav_pub.publish(Bool(data=True))

    def _drive_assist(self, msg):
        # While a maneuver is running the timer drives — ignore stick.
        if self._assist_state != 'drive':
            return

        twist = Twist()
        twist.linear.x  = msg.axes[AXIS_LEFT_UD] * 0.5
        twist.angular.z = msg.axes[AXIS_LEFT_LR] * 1.0

        if twist.linear.x > 0.0:
            front = self._sector_min_range(0.0, FRONT_CONE_HALF_DEG)
            if front < OBSTACLE_TRIGGER_DIST:
                self._assist_state = 'turn'
                self._maneuver_start = self._now_sec()
                side = 'RIGHT' if AVOID_DIR < 0 else 'LEFT'
                self.get_logger().warn(
                    f'Obstacle {front:.2f} m → 90° {side} then 1.5 m arc')
                self.cmd_vel_pub.publish(Twist())
                return

        self.cmd_vel_pub.publish(twist)

    def _assist_tick(self):
        if self.mode != 'assist' or self._assist_state == 'drive':
            return

        elapsed = self._now_sec() - self._maneuver_start
        twist = Twist()

        if self._assist_state == 'turn':
            if elapsed >= TURN_DURATION_S:
                self._assist_state = 'arc'
                self._maneuver_start = self._now_sec()
                self.get_logger().info('Turn done → arcing around obstacle')
                return
            twist.angular.z = AVOID_DIR * TURN_ANGULAR
            self.cmd_vel_pub.publish(twist)
            return

        if self._assist_state == 'arc':
            if elapsed >= ARC_DURATION_S:
                self._assist_state = 'return'
                self._maneuver_start = self._now_sec()
                self.get_logger().info('Arc done → turning to recover heading')
                return
            # If obstacle reappears during arc, bail early to 'return' state
            front = self._sector_min_range(0.0, FRONT_CONE_HALF_DEG)
            if front < OBSTACLE_TRIGGER_DIST:
                self._assist_state = 'return'
                self._maneuver_start = self._now_sec()
                self.get_logger().warn(
                    f'Obstacle re-detected at {front:.2f} m during arc → early exit')
                return
            twist.linear.x = ARC_LINEAR
            # Arc curves back toward the object (opposite of the initial turn)
            twist.angular.z = -AVOID_DIR * ARC_ANGULAR
            self.cmd_vel_pub.publish(twist)
            return

        if self._assist_state == 'return':
            # After 90° initial turn (AVOID_DIR) + 180° arc (-AVOID_DIR), the net
            # heading change is -AVOID_DIR * 90°. One more 90° turn in AVOID_DIR
            # restores the original heading.
            if elapsed >= TURN_DURATION_S:
                self._assist_state = 'drive'
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().info('Heading restored → stick drive')
                return
            twist.angular.z = AVOID_DIR * TURN_ANGULAR
            self.cmd_vel_pub.publish(twist)

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _enforce_deadman(self):
        if self.mode == 'auto' and not self.deadman_pressed:
            self.cmd_vel_pub.publish(Twist())

    def _sector_min_range(self, center_deg, half_deg):
        if self._scan is None or not self._scan.ranges:
            return float('inf')

        center_rad = math.radians(center_deg)
        half_rad = math.radians(half_deg)
        rmin = self._scan.range_min if self._scan.range_min > 0 else 0.05
        rmax = self._scan.range_max if self._scan.range_max > 0 else 30.0

        min_r = float('inf')
        angle = self._scan.angle_min
        for r in self._scan.ranges:
            diff = (angle - center_rad + math.pi) % (2 * math.pi) - math.pi
            if abs(diff) <= half_rad and rmin < r < rmax and math.isfinite(r):
                min_r = min(min_r, r)
            angle += self._scan.angle_increment
        return min_r

    def _scan_cb(self, msg):
        self._scan = msg

    def _gps_nav_vel_cb(self, msg):
        if self.mode == 'auto' and self.deadman_pressed:
            self.cmd_vel_pub.publish(msg)

    def _check_controller(self):
        if self.last_joy_time is None:
            return
        elapsed = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        if elapsed > 2.0 and self.controller_connected:
            self.controller_connected = False
            self.get_logger().warn('Controller DISCONNECTED')
            self.cmd_vel_pub.publish(Twist())

    def _publish_mode(self):
        self.mode_pub.publish(String(data=self.mode))


def main(args=None):
    rclpy.init(args=args)
    node = GamepadController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
