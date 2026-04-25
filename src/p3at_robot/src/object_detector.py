#!/usr/bin/env python3
"""
Object detector — color-contour detection on /camera/rgb/image_raw.

Finds cone-candidate blobs (red / orange / yellow / white) with grass rejection
and ground-contact filtering (rejects blobs floating in upper part of frame,
which are usually sky / background). Distance comes from the LIDAR ray at the
blob's bearing, since oak_camera.py publishes RGB only.

Provides a hysteresis-filtered /vision/cone_confirmed signal so the assist-mode
avoidance in gamepad_controller.py only triggers on consistent detections, not
single-frame false positives.

Topics:
  /camera/rgb/image_raw    — BGR input (subscribed)
  /scan                    — LIDAR for distance (subscribed)
  /vision/objects          — JSON list of detections (published)
  /vision/cone_confirmed   — Bool, confirmed cone-coloured blob in front (published)
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

CAMERA_HFOV_DEG = 66.0   # OAK default — adjust if your FOV differs

# Contour filters — matched to the camera_smoke_test.py invocation the user
# validated on-robot, scaled to the 320x240 stream from oak_camera.py.
MIN_BOTTOM_FRAC  = 0.22   # blob bottom must be in lower 78% of frame
MIN_ASPECT_RATIO = 0.28   # w/h
MAX_ASPECT_RATIO = 6.0
MIN_AREA_FRAC    = 0.0055 # ≈ 1400 px at 640x400 → ~420 px at 320x240
MAX_GRASS_RATIO  = 0.05   # reject if >5% of blob pixels look like grass
MORPH_KERNEL     = 7

# HSV colour ranges (OpenCV: H in 0-180)
COLOR_RANGES = {
    'red':    [((0, 100, 80),   (8, 255, 255)),
               ((170, 100, 80), (180, 255, 255))],
    'orange': [((8, 120, 100),  (22, 255, 255))],
    'yellow': [((22, 100, 100), (35, 255, 255))],
    'white':  [((0, 0, 180),    (180, 40, 255))],
}
GRASS_RANGE = ((30, 40, 30), (90, 255, 230))

# Cone confirmation
CONE_COLOURS        = {'orange', 'red'}
CONE_FRONT_DEG      = 30.0   # only count cones within this bearing as "in front"
CONE_CONFIRM_FRAMES = 3      # consecutive hits needed to assert confirmation
CONE_LOSE_FRAMES    = 5      # consecutive misses needed to drop confirmation


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        self.bridge = CvBridge()
        self._scan = None
        self._hits = 0
        self._misses = 0
        self._confirmed = False

        self.create_subscription(Image, '/camera/rgb/image_raw', self._image_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

        self.objects_pub = self.create_publisher(String, '/vision/objects', 10)
        self.cone_pub    = self.create_publisher(Bool,   '/vision/cone_confirmed', 10)

        self.get_logger().info('Object detector ready — listening to /camera/rgb/image_raw')

    def _scan_cb(self, msg):
        self._scan = msg

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        detections = self._detect(frame)
        self.objects_pub.publish(String(data=json.dumps({'detections': detections})))
        self._update_cone_confirmation(detections)

    def _detect(self, frame):
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        candidate = np.zeros((h, w), dtype=np.uint8)
        for ranges in COLOR_RANGES.values():
            for low, high in ranges:
                candidate = cv2.bitwise_or(
                    candidate,
                    cv2.inRange(hsv, np.array(low), np.array(high)))

        grass = cv2.inRange(hsv, np.array(GRASS_RANGE[0]), np.array(GRASS_RANGE[1]))
        candidate = cv2.bitwise_and(candidate, cv2.bitwise_not(grass))

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (MORPH_KERNEL, MORPH_KERNEL))
        candidate = cv2.morphologyEx(candidate, cv2.MORPH_OPEN, kernel)
        candidate = cv2.morphologyEx(candidate, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(candidate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = MIN_AREA_FRAC * w * h

        detections = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < min_area:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            if bh == 0:
                continue
            aspect = bw / float(bh)
            if aspect < MIN_ASPECT_RATIO or aspect > MAX_ASPECT_RATIO:
                continue
            if (y + bh) / float(h) < MIN_BOTTOM_FRAC:
                continue  # blob floats high in frame → probably sky / trees

            blob_mask = np.zeros((h, w), dtype=np.uint8)
            cv2.drawContours(blob_mask, [c], -1, 255, -1)
            blob_pixels = cv2.countNonZero(blob_mask)
            if blob_pixels == 0:
                continue
            grass_pixels = cv2.countNonZero(cv2.bitwise_and(grass, blob_mask))
            if grass_pixels / float(blob_pixels) > MAX_GRASS_RATIO:
                continue

            mean_hsv = cv2.mean(hsv, mask=blob_mask)
            colour = self._classify_colour(mean_hsv[0], mean_hsv[1], mean_hsv[2])

            cx = x + bw / 2.0
            bearing_deg = -(cx - w / 2.0) / (w / 2.0) * (CAMERA_HFOV_DEG / 2.0)
            distance_m = self._lidar_range_at(math.radians(bearing_deg))

            detections.append({
                'colour': colour,
                'bearing_deg': float(bearing_deg),
                'distance_m': float(distance_m),
                'area_px': float(area),
                'bbox': [int(x), int(y), int(bw), int(bh)],
            })

        return detections

    def _classify_colour(self, hue, sat, val):
        if sat < 60:
            return 'white' if val > 180 else 'grey'
        if hue < 10 or hue >= 170:
            return 'red'
        if hue < 22:
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

    def _update_cone_confirmation(self, detections):
        cone_in_front = any(
            d['colour'] in CONE_COLOURS and abs(d['bearing_deg']) <= CONE_FRONT_DEG
            for d in detections)

        if cone_in_front:
            self._hits += 1
            self._misses = 0
            if not self._confirmed and self._hits >= CONE_CONFIRM_FRAMES:
                self._confirmed = True
                self.get_logger().info('Cone CONFIRMED')
        else:
            self._misses += 1
            self._hits = 0
            if self._confirmed and self._misses >= CONE_LOSE_FRAMES:
                self._confirmed = False
                self.get_logger().info('Cone lost')

        self.cone_pub.publish(Bool(data=self._confirmed))


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
