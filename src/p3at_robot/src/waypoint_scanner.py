#!/usr/bin/env python3
"""
Waypoint scanner — 360° sweep to find a random marker object on a grass field.

Triggered by /waypoint_scan/start (String = waypoint label). The navigator
rotates the robot in place; this node processes camera frames during the sweep,
finds the most prominent non-grass, non-orange object, and publishes an
annotated image plus distance (from LIDAR) to Foxglove.

Topics:
  /camera/rgb/image_raw      — RGB input (subscribed)
  /scan                      — LIDAR for distance (subscribed)
  /waypoint_scan/start       — String: waypoint label, starts a scan (subscribed)
  /waypoint_scan/object_image — annotated best detection (published)
  /waypoint_scan/object_info — JSON: label, bearing_deg, distance_m, area (published)
  /waypoint_scan/complete    — Bool: scan finished (published)
"""
import json
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

# Scan timing — should be >= navigator rotation duration
SCAN_DURATION_S = 20.0

# Detection thresholds (HSV)
GRASS_H_LOW, GRASS_H_HIGH   = 30, 90      # green grass hue range
GRASS_S_MIN = 40
ORANGE_H_LOW, ORANGE_H_HIGH = 5, 25       # cone hue range (ignore)
ORANGE_S_MIN = 120
MIN_AREA_PX = 400                         # ignore tiny blobs

# Camera geometry (OAK default — adjust if you know your actual HFOV)
CAMERA_HFOV_DEG = 66.0


class WaypointScanner(Node):
    def __init__(self):
        super().__init__('waypoint_scanner')

        self.bridge = CvBridge()
        self._frame = None
        self._scan = None
        self._active = False
        self._scan_label = ''
        self._scan_end_time = 0.0

        # Best detection over the sweep
        self._best = None   # dict: area, bearing_deg, distance_m, annotated_bgr

        self.create_subscription(Image, '/camera/rgb/image_raw', self._image_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(String, '/waypoint_scan/start', self._start_cb, 10)

        self.image_pub    = self.create_publisher(Image,  '/waypoint_scan/object_image', 10)
        self.info_pub     = self.create_publisher(String, '/waypoint_scan/object_info',  10)
        self.complete_pub = self.create_publisher(Bool,   '/waypoint_scan/complete',     10)

        self.create_timer(0.2, self._tick)   # 5 Hz processing

        self.get_logger().info('Waypoint scanner ready — waiting for /waypoint_scan/start')

    def _image_cb(self, msg):
        try:
            self._frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')

    def _scan_cb(self, msg):
        self._scan = msg

    def _start_cb(self, msg):
        self._active = True
        self._scan_label = msg.data
        self._scan_end_time = self.get_clock().now().nanoseconds / 1e9 + SCAN_DURATION_S
        self._best = None
        self.get_logger().info(f'Scan started for "{self._scan_label}" ({SCAN_DURATION_S:.0f}s)')

    def _tick(self):
        if not self._active:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now >= self._scan_end_time:
            self._finalize()
            return

        if self._frame is None:
            return

        det = self._detect(self._frame)
        if det is not None and (self._best is None or det['area'] > self._best['area']):
            self._best = det

    def _detect(self, frame):
        """Find the biggest non-grass, non-orange blob. Returns dict or None."""
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        grass = cv2.inRange(hsv,
                            np.array([GRASS_H_LOW,  GRASS_S_MIN, 30]),
                            np.array([GRASS_H_HIGH, 255,        230]))
        orange = cv2.inRange(hsv,
                             np.array([ORANGE_H_LOW,  ORANGE_S_MIN, 60]),
                             np.array([ORANGE_H_HIGH, 255,          255]))

        # Candidate = anything that's not grass and not orange, with enough saturation/value
        # to reject sky/shadows
        not_grass = cv2.bitwise_not(grass)
        not_orange = cv2.bitwise_not(orange)
        mask = cv2.bitwise_and(not_grass, not_orange)

        # Reject very dark / very washed-out pixels (shadows, overexposed sky)
        v_ok = cv2.inRange(hsv[:, :, 2], 40, 240)
        mask = cv2.bitwise_and(mask, v_ok)

        # Clean up
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        biggest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(biggest)
        if area < MIN_AREA_PX:
            return None

        x, y, bw, bh = cv2.boundingRect(biggest)
        cx = x + bw / 2.0

        # Pixel x → bearing in robot frame (0 = forward, +left)
        bearing_deg = -(cx - w / 2.0) / (w / 2.0) * (CAMERA_HFOV_DEG / 2.0)

        distance_m = self._lidar_range_at(math.radians(bearing_deg))
        shape = self._classify_shape(biggest, area, bw, bh)

        # Dominant colour (mean HSV hue inside the contour)
        blob_mask = np.zeros(mask.shape, dtype=np.uint8)
        cv2.drawContours(blob_mask, [biggest], -1, 255, -1)
        mean_hsv = cv2.mean(hsv, mask=blob_mask)
        colour = self._classify_colour(mean_hsv[0], mean_hsv[1])

        annotated = frame.copy()
        cv2.rectangle(annotated, (x, y), (x + bw, y + bh), (0, 255, 255), 3)
        label = f'{colour} {shape} {bearing_deg:+.0f}deg {distance_m:.2f}m'
        cv2.putText(annotated, label, (x, max(20, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        return {
            'area': float(area),
            'bearing_deg': float(bearing_deg),
            'distance_m': float(distance_m),
            'shape': shape,
            'colour': colour,
            'annotated_bgr': annotated,
        }

    def _classify_shape(self, contour, area, bw, bh):
        """Classify contour shape via vertex count + circularity."""
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            return 'unknown'

        # Circularity: 1.0 for a perfect circle, lower for other shapes
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        if circularity > 0.80:
            return 'circle'

        approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
        v = len(approx)
        if v == 3:
            return 'triangle'
        if v == 4:
            aspect = bw / float(bh) if bh > 0 else 1.0
            return 'square' if 0.85 < aspect < 1.15 else 'rectangle'
        if v == 5:
            return 'pentagon'
        if v == 6:
            return 'hexagon'
        if circularity > 0.65:
            return 'circle'
        return f'polygon({v})'

    def _classify_colour(self, hue, sat):
        """Rough HSV → colour name. Hue is 0-180 in OpenCV."""
        if sat < 60:
            return 'grey'
        if hue < 10 or hue >= 170:
            return 'red'
        if hue < 25:
            return 'orange'
        if hue < 35:
            return 'yellow'
        if hue < 85:
            return 'green'
        if hue < 130:
            return 'blue'
        if hue < 160:
            return 'purple'
        return 'pink'

    def _lidar_range_at(self, angle_rad, half_cone_rad=math.radians(3.0)):
        scan = self._scan
        if scan is None or not scan.ranges:
            return float('inf')
        rmin = scan.range_min if scan.range_min > 0.0 else 0.05
        rmax = scan.range_max if scan.range_max > 0.0 else 30.0
        best = float('inf')
        a = scan.angle_min
        inc = scan.angle_increment
        for r in scan.ranges:
            da = math.atan2(math.sin(a - angle_rad), math.cos(a - angle_rad))
            if abs(da) <= half_cone_rad and rmin < r < rmax and math.isfinite(r):
                if r < best:
                    best = r
            a += inc
        return best

    def _finalize(self):
        self._active = False

        if self._best is None:
            self.get_logger().info(f'Scan "{self._scan_label}" finished — no object found')
            info = {'label': self._scan_label, 'detected': False}
            self.info_pub.publish(String(data=json.dumps(info)))
        else:
            b = self._best
            info = {
                'label': self._scan_label,
                'detected': True,
                'shape': b.get('shape', 'unknown'),
                'colour': b.get('colour', 'unknown'),
                'bearing_deg': b['bearing_deg'],
                'distance_m': b['distance_m'],
                'area_px': b['area'],
            }
            self.info_pub.publish(String(data=json.dumps(info)))
            img_msg = self.bridge.cv2_to_imgmsg(b['annotated_bgr'], 'bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_rgb'
            self.image_pub.publish(img_msg)
            self.get_logger().info(
                f'Scan "{self._scan_label}" done — {b.get("colour","?")} '
                f'{b.get("shape","?")} at {b["bearing_deg"]:+.1f}°, {b["distance_m"]:.2f}m')

        self.complete_pub.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = WaypointScanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
