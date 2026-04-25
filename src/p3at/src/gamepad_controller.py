#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Button mapping
BTN_X       = 0
BTN_CIRCLE  = 1
BTN_SQUARE  = 2
BTN_TRIANGLE= 3

# Axes mapping
AXIS_LEFT_LR  = 0   # left stick left/right
AXIS_LEFT_UD  = 1   # left stick up/down
AXIS_L2       = 4   # left trigger  (1.0=released, -1.0=fully pressed)
AXIS_R2       = 5   # right trigger (1.0=released, -1.0=fully pressed)

DEADMAN_THRESHOLD = 0.0   # trigger counts as pressed when below this value

class GamepadController(Node):
    def __init__(self):
        super().__init__('gamepad_controller')

        self.mode = 'manual'   # start in manual mode
        self.deadman_pressed = False

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Publishes drive commands in manual mode
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publishes current mode so other nodes know what's happening
        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)

        self.get_logger().info('Gamepad controller started in MANUAL mode')
        self.get_logger().info('O = manual mode | X = automated mode')

    def joy_callback(self, msg):
        # --- Mode switching ---
        if msg.buttons[BTN_X] == 1 and self.mode != 'auto':
            self.mode = 'auto'
            self.get_logger().info('Switched to AUTOMATED mode - hold both triggers as dead-man switch')
            self.publish_mode()

        if msg.buttons[BTN_CIRCLE] == 1 and self.mode != 'manual':
            self.mode = 'manual'
            self.get_logger().info('Switched to MANUAL mode')
            self.publish_mode()

        # --- Manual driving ---
        if self.mode == 'manual':
            self.drive_manual(msg)

        # --- Automated mode dead-man switch ---
        elif self.mode == 'auto':
            l2 = msg.axes[AXIS_L2]
            r2 = msg.axes[AXIS_R2]
            both_pressed = (l2 < DEADMAN_THRESHOLD) and (r2 < DEADMAN_THRESHOLD)

            if not both_pressed:
                # Dead-man released — stop robot immediately
                self.cmd_vel_pub.publish(Twist())
                if self.deadman_pressed:
                    self.get_logger().warn('Dead-man switch released! Stopping robot.')
            self.deadman_pressed = both_pressed
            self.publish_mode()

    def drive_manual(self, msg):
        twist = Twist()
        twist.linear.x  = msg.axes[AXIS_LEFT_UD] * 0.5   # max 0.5 m/s
        twist.angular.z = msg.axes[AXIS_LEFT_LR] * 1.0   # max 1.0 rad/s
        self.cmd_vel_pub.publish(twist)

    def publish_mode(self):
        msg = String()
        msg.data = self.mode
        self.mode_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GamepadController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
