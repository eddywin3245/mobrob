#!/usr/bin/env python3
"""
Magnetometer hard-iron calibration.

Usage:
  1. Start the full launch (gamepad.launch.py) so /imu/mag is publishing.
  2. In a second terminal run:
       ros2 run p3at_robot mag_calibrate.py
  3. Slowly rotate the robot at least ONE full 360° turn (2 turns is better).
  4. Press Ctrl+C when done.
  5. Copy the printed MAG_X_OFFSET / MAG_Y_OFFSET values into
     gps_waypoint_navigator.py under '# Edit before each run'.

What it does: records the full ellipse traced by (mag_x, mag_y) as the
robot rotates, finds the centre of that ellipse (hard-iron offset), and
removes the robot's own magnetic bias so Earth's field dominates.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField


class MagCalNode(Node):
    def __init__(self):
        super().__init__('mag_calibrate')
        self.samples = []
        self.create_subscription(MagneticField, '/imu/mag', self._cb, 10)
        self.get_logger().info(
            'Collecting magnetometer data — rotate the robot slowly ONE full 360°, then Ctrl+C')

    def _cb(self, msg):
        self.samples.append((msg.magnetic_field.x, msg.magnetic_field.y))
        if len(self.samples) % 100 == 0:
            self.get_logger().info(f'  {len(self.samples)} samples...')


def main():
    rclpy.init()
    node = MagCalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    samples = node.samples
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass  # ignore double-shutdown from SIGINT

    if len(samples) < 100:
        print(f'Only {len(samples)} samples — not enough. Rotate the robot more.')
        return

    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]

    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)

    x_offset = (x_max + x_min) / 2.0
    y_offset = (y_max + y_min) / 2.0

    print(f'\n{"="*52}')
    print(f'Calibration complete  ({len(samples)} samples)')
    print(f'{"="*52}')
    print(f'X: min={x_min:.4e}  max={x_max:.4e}  offset={x_offset:.4e}')
    print(f'Y: min={y_min:.4e}  max={y_max:.4e}  offset={y_offset:.4e}')
    print()
    print('Copy these two lines into gps_waypoint_navigator.py:')
    print(f'  MAG_X_OFFSET = {x_offset:.6e}')
    print(f'  MAG_Y_OFFSET = {y_offset:.6e}')
    print()

    # Quick sanity-check: heading at the last orientation
    recent = samples[-30:]
    avg_x = sum(s[0] - x_offset for s in recent) / len(recent)
    avg_y = sum(s[1] - y_offset for s in recent) / len(recent)
    # Must match the formula in gps_waypoint_navigator.py:_heading_deg()
    heading = (math.degrees(math.atan2(avg_x, avg_y)) + 1.4) % 360
    print(f'Heading at end of rotation: {heading:.1f} deg')
    print('Point robot north and re-run to verify it reads ~0°.')


if __name__ == '__main__':
    main()
