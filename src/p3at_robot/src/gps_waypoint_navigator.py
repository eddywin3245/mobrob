#!/usr/bin/env python3
"""
GPS waypoint navigator — no Nav2/SLAM dependency.

Uses magnetometer for heading, drives straight to each GPS waypoint. If the
LIDAR sees any obstacle in the front cone, the robot executes a 90° turn →
semicircle arc around the obstacle → 90° return turn so it ends up past the
obstacle facing its original heading. Arc radius matches the detection
distance, so the robot stays ~r away from the obstacle throughout. If a new
obstacle appears during the arc, the robot stops until the way is clear for
2 s, then resumes the arc where it left off.

Press Square on gamepad (auto mode, R2 held) to start.

Foxglove debug topics (always publishing):
  /gps_nav/heading_deg   Float32  magnetometer heading (Plot panel)
  /gps_nav/mag_x_raw     Float32  raw compensated mag X (Plot panel)
  /gps_nav/mag_y_raw     Float32  raw compensated mag Y (Plot panel)

Foxglove debug topics (active during navigation):
  /gps_nav/target_fix    NavSatFix  target waypoint (Map panel — pin on map)
  /gps_nav/bearing_deg   Float32    bearing to goal (Plot panel)
  /gps_nav/error_deg     Float32    heading error, +ve = turn right (Plot panel)
  /gps_nav/distance_m    Float32    distance to goal in metres (Plot panel)
  /gps_nav/status        String     human-readable status
"""
import math
import os
import subprocess
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32
from sensor_msgs.msg import NavSatFix, MagneticField, LaserScan, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from collections import deque

# ── Edit before each run ─────────────────────────────────────────────────────

GPS_WAYPOINTS = [
    # Example waypoints in Perth, WA — replace with your own!
    (-31.9804058, 115.8178087),  # Point 1
    (-31.9799439, 115.8175824),  # Point 2,
    (-31.9801936, 115.8173269),
]

# Hard-iron calibration offsets
MAG_X_OFFSET = 8.302800e-05
MAG_Y_OFFSET = 4.882000e-05

# Magnetic declination for Perth, WA
MAGNETIC_DECLINATION_DEG = 1.4

# Navigation tuning
GOAL_RADIUS_M     = 0.5   # stop when within this distance of waypoint
WAYPOINT_PAUSE_S  = 3.0   # seconds to stop at each waypoint before continuing
LINEAR_SPEED      = 1.5   # m/s forward speed
MAX_ANGULAR       = 1.2   # rad/s max turn rate

# PID Tuning — Heading control (GPS waypoint navigation)
# Proportional gain: higher = more aggressive turning, lower = smoother
ANGULAR_GAIN      = 2.5   # P gain for heading error → angular velocity (rad/s per degree error)
ANGULAR_DAMPING   = 1.1   # D gain for heading rate (dampens oscillation)

# Cone approach — takes over from GPS nav within this distance of waypoint
CONE_APPROACH_GPS_DIST_M   = 3.0   # within this GPS distance, check for cone near waypoint
CONE_OBJECT_MAX_WP_DIST_M  = 5.0   # ignore LIDAR object if estimated >this far from waypoint

# Arc-around-obstacle avoidance (tuned values from gamepad_controller.py)
OBSTACLE_TRIGGER_DIST = 0.8    # front-cone LIDAR closer than this → arc maneuver
FRONT_CONE_HALF_DEG   = 30.0   # ±30° front cone for trigger + blocked check
AVOID_DIR             = -1     # -1 = pass on the right, +1 = pass on the left (first avoidance)
ATTRACT_GAIN          = 0.4    # angular bias (rad/s per rad offset) toward nearest front object
TURN_ANGULAR          = 0.5    # rad/s during 90° rotations
TURN_DURATION_S       = math.radians(90) / 0.5   # ≈ 3.14 s for 90°
ARC_LINEAR            = 0.3    # m/s forward speed during semicircle
ARC_RADIUS            = 2    # m (fixed radius, or use detection distance if smaller)

# GPS filtering + odometry fusion
GPS_OUTLIER_DIST_M = 8.0   # reject GPS reads >5m from recent median
GPS_BUFFER_SIZE    = 30    # keep last 30 GPS samples (~30 seconds at 1 Hz)
WAYPOINT_VERIFY_S  = 3.0   # wait this long before declaring waypoint reached
WAYPOINT_CLUSTER_M = 1.5   # GPS must stay within this radius during verify window
ODOM_GPS_AGREE_M   = 2.0   # odom and GPS must agree within this distance

# Waypoint inspection — runs after each waypoint is reached
INSPECTION_CONE_APPROACH_M   = 1.5    # drive to this distance from cone marker
INSPECTION_SIDE_OFFSET_M     = 1.2    # drive forward this far once beside the cone
INSPECTION_ALIGN_SPEED_RAD   = 0.5    # rad/s for in-place heading alignment
INSPECTION_APPROACH_STOP_M   = 1.5    # stop this far from secondary object
INSPECTION_PHOTO_COUNT       = 5      # how many photos to capture
INSPECTION_PHOTO_DIR         = '/data/inspection_photos'
BAG_OUTPUT_DIR               = '/data/ros_bags'
INSPECTION_MIN_OBJECT_DIST_M = 0.3    # ignore LIDAR returns closer than this (noise)
INSPECTION_MAX_OBJECT_DIST_M = 8.0    # max range to consider objects

# ─────────────────────────────────────────────────────────────────────────────


def haversine_distance(lat1, lon1, lat2, lon2):
    """Straight-line distance in metres between two GPS coordinates."""
    R = 6371000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing_to(lat1, lon1, lat2, lon2):
    """Bearing in degrees (0=N, 90=E) from point 1 to point 2."""
    dlon = math.radians(lon2 - lon1)
    lat1r, lat2r = math.radians(lat1), math.radians(lat2)
    x = math.sin(dlon) * math.cos(lat2r)
    y = math.cos(lat1r) * math.sin(lat2r) - math.sin(lat1r) * math.cos(lat2r) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def angle_diff(a, b):
    """Signed difference between angles a and b in degrees (-180 to 180)."""
    return (a - b + 180) % 360 - 180


def offset_gps(lat, lon, dist_m, bearing_deg):
    """Return GPS coords offset dist_m in direction bearing_deg from lat,lon."""
    R = 6371000.0
    lat2 = math.degrees(math.asin(
        math.sin(math.radians(lat)) * math.cos(dist_m / R)
        + math.cos(math.radians(lat)) * math.sin(dist_m / R)
        * math.cos(math.radians(bearing_deg))
    ))
    lon2 = lon + math.degrees(math.atan2(
        math.sin(math.radians(bearing_deg)) * math.sin(dist_m / R)
        * math.cos(math.radians(lat)),
        math.cos(dist_m / R) - math.sin(math.radians(lat)) * math.sin(math.radians(lat2))
    ))
    return lat2, lon2


def _detect_shape(cnt, color):
    import cv2
    area = cv2.contourArea(cnt)
    perimeter = cv2.arcLength(cnt, True)
    if perimeter == 0:
        return 'unknown'
    circularity = 4 * math.pi * (area / (perimeter * perimeter))
    approx = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)
    sides = len(approx)
    x, y, w, h = cv2.boundingRect(cnt)
    aspect_ratio = h / float(w)

    if sides > 7 and color == 'yellow':
        return 'irregular'
    if circularity > 0.8:
        return 'circle'
    if aspect_ratio > 1.3 and circularity < 0.8:
        return 'cone'
    if sides == 4:
        return 'bucket'
    if circularity < 0.8:
        return 'tub'
    return 'irregular'


def _detect_col(frame):
    import cv2
    image = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    max_area = 0
    detected = None

    hues = {
        'orange': [(5, 19)],
        'yellow': [(20, 31)],
        'green':  [(40, 90)],
        'red':    [(0, 10), (170, 179)],
    }

    for color, ranges in hues.items():
        for (low_h, high_h) in ranges:
            lower = np.array([low_h, 90, 50])
            upper = np.array([high_h, 255, 255])
            mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((5, 5), np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 1000:
                    continue
                perimeter = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)
                if len(approx) > 10:
                    continue
                shape = _detect_shape(cnt, color)
                if shape == 'irregular':
                    continue
                if area > max_area:
                    max_area = area
                    detected = (color, cnt, shape)

    lower_white = np.array([0, 0, 200])
    upper_white = np.array([179, 50, 255])
    mask = cv2.inRange(hsv, lower_white, upper_white)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 1000:
            continue
        perimeter = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)
        if len(approx) > 10:
            continue
        shape = _detect_shape(cnt, 'white')
        if shape == 'irregular':
            continue
        if area > max_area:
            max_area = area
            detected = ('white', cnt, 'helmet')

    return detected


class GpsWaypointNavigator(Node):
    def __init__(self):
        super().__init__('gps_waypoint_navigator')

        self._gps_fix = None
        self._mag_samples = []
        self._scan = None
        self._navigating = False
        self._cancel_requested = False
        self._last_trigger_time = 0.0

        # GPS filtering: rolling buffer of (lat, lon, timestamp) to reject outliers
        self._gps_buffer = deque(maxlen=GPS_BUFFER_SIZE)

        # Odometry: latest pose for fusion with GPS and turn control
        self._odom_pos = None  # (x, y) in map frame
        self._odom_msg = None  # full Odometry message for yaw extraction

        # Heading error tracking for D term (rate of error change)
        self._last_heading_error_deg = None
        self._last_error_time = 0.0

        # Latest camera frame — populated by _image_cb, used by _take_photos
        self._latest_image = None
        self._bag_proc = None
        self._inspection_results = []
        self._stopped_at_cone = False
        self._avoid_dir = AVOID_DIR   # flips sign after each completed avoidance

        self.create_subscription(NavSatFix,     '/fix',                   self._gps_cb,    10)
        self.create_subscription(MagneticField, '/imu/mag',               self._mag_cb,    10)
        self.create_subscription(LaserScan,     '/scan',                  self._scan_cb,   10)
        self.create_subscription(Bool,          '/start_gps_nav',         self._trigger,   10)
        self.create_subscription(Odometry,      '/odom',                  self._odom_cb,   10)
        self.create_subscription(Image,         '/camera/rgb/image_raw',  self._image_cb,  1)

        # Drive output — gamepad forwards this to /cmd_vel only when deadman held
        self._cmd_pub = self.create_publisher(Twist, '/gps_nav/cmd_vel', 10)

        # ── Foxglove debug publishers ─────────────────────────────────────────
        self._target_fix_pub  = self.create_publisher(NavSatFix, '/gps_nav/target_fix',  10)
        self._current_fix_pub = self.create_publisher(NavSatFix, '/gps_nav/current_fix', 10)

        # One publisher per waypoint so all pins show on Foxglove Map simultaneously
        self._waypoint_pubs = [
            self.create_publisher(NavSatFix, f'/gps_nav/waypoint_{i+1}', 10)
            for i in range(len(GPS_WAYPOINTS))
        ]

        self._active_pub   = self.create_publisher(Bool,    '/gps_nav/active',      10)
        self._heading_pub  = self.create_publisher(Float32, '/gps_nav/heading_deg', 10)
        self._bearing_pub  = self.create_publisher(Float32, '/gps_nav/bearing_deg', 10)
        self._error_pub    = self.create_publisher(Float32, '/gps_nav/error_deg',   10)
        self._distance_pub = self.create_publisher(Float32, '/gps_nav/distance_m',  10)
        self._mag_x_pub    = self.create_publisher(Float32, '/gps_nav/mag_x_raw',   10)
        self._mag_y_pub    = self.create_publisher(Float32, '/gps_nav/mag_y_raw',   10)
        self._status_pub   = self.create_publisher(String,  '/gps_nav/status',      10)

        self.create_timer(0.2, self._publish_heading_debug)
        self.create_timer(1.0, self._publish_all_waypoints)

        self.get_logger().info('GPS navigator ready — Foxglove topics active on /gps_nav/*')
        self.get_logger().info('Press Square (auto mode) to start.')

    # ── Sensor callbacks ──────────────────────────────────────────────────────

    def _gps_cb(self, msg):
        if msg.status.status >= 0:
            # Soft filter: warn on outliers but still use them (don't hard-reject)
            # This prevents wild jumps from derailing navigation but allows good reads through
            median = self._gps_median()
            if median is not None:
                d = haversine_distance(median[0], median[1], msg.latitude, msg.longitude)
                if d > GPS_OUTLIER_DIST_M:
                    self.get_logger().warn(
                        f'GPS potential outlier: {d:.1f}m from median (still using)')

            # Always add to buffer and update _gps_fix for smooth navigation
            self._gps_buffer.append((msg.latitude, msg.longitude, time.monotonic()))
            self._gps_fix = msg
            self._current_fix_pub.publish(msg)

    def _mag_cb(self, msg):
        self._mag_samples.append((msg.magnetic_field.x, msg.magnetic_field.y))
        if len(self._mag_samples) > 50:
            self._mag_samples.pop(0)

    def _scan_cb(self, msg):
        self._scan = msg



    def _odom_cb(self, msg):
        """Cache odometry position and orientation for GPS fusion and turn control."""
        self._odom_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self._odom_msg = msg

    def _image_cb(self, msg):
        self._latest_image = msg

    def _gps_median(self):
        """Return median GPS position from recent buffer, or None if buffer empty."""
        if not self._gps_buffer:
            return None
        lats = sorted([lat for lat, lon, ts in self._gps_buffer])
        lons = sorted([lon for lat, lon, ts in self._gps_buffer])
        mid = len(lats) // 2
        return lats[mid], lons[mid]

    def _gps_cluster_radius(self):
        """Return radius of current GPS cluster (max distance from median), or inf."""
        median = self._gps_median()
        if median is None:
            return float('inf')
        med_lat, med_lon = median
        max_dist = 0.0
        for lat, lon, ts in self._gps_buffer:
            d = haversine_distance(med_lat, med_lon, lat, lon)
            max_dist = max(max_dist, d)
        return max_dist

    def _find_nearest_lidar_object_any_direction(self):
        """Nearest LIDAR return in the full 360°, or None if nothing in range."""
        scan = self._scan
        if scan is None or not scan.ranges:
            return None
        rmin = max(scan.range_min if scan.range_min > 0.0 else 0.05,
                   INSPECTION_MIN_OBJECT_DIST_M)
        rmax = min(scan.range_max if scan.range_max > 0.0 else 30.0,
                   INSPECTION_MAX_OBJECT_DIST_M)
        closest_range = float('inf')
        closest_angle = None
        a = scan.angle_min
        for r in scan.ranges:
            if not (math.isinf(r) or math.isnan(r)) and rmin <= r <= rmax:
                if r < closest_range:
                    closest_range = r
                    closest_angle = a
            a += scan.angle_increment
        if closest_angle is None:
            return None
        return {'bearing_deg': math.degrees(closest_angle), 'distance_m': closest_range}

    # ── Inspection movement primitives ────────────────────────────────────────

    def _align_to_world_bearing(self, target_deg, tol_deg=5.0, timeout_s=12.0):
        """Rotate in place until robot compass heading matches target_deg.

        Uses only the 3 most recent magnetometer samples so heading feedback
        is responsive during the turn (no lag from the long moving average).
        """
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            if self._cancel_requested:
                self._stop()
                return
            result = self._heading_deg(n_samples=3)
            if result is None:
                time.sleep(0.1)
                continue
            heading, _, _ = result
            err = angle_diff(target_deg, heading)
            if abs(err) <= tol_deg:
                self._stop()
                return
            twist = Twist()
            # P controller: positive error → need to turn CW → angular.z < 0
            twist.angular.z = max(-INSPECTION_ALIGN_SPEED_RAD,
                                  min(INSPECTION_ALIGN_SPEED_RAD,
                                      -math.radians(err) * 2.0))
            self._cmd_pub.publish(twist)
            time.sleep(0.1)
        self._stop()

    def _odom_yaw_deg(self):
        """Current yaw from wheel-odometry quaternion in degrees (CCW positive)."""
        if self._odom_msg is None:
            return None
        q = self._odom_msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.degrees(math.atan2(siny, cosy))

    def _rotate_by_angle_deg(self, angle_deg, speed=0.4):
        """Rotate by angle_deg using odometry yaw feedback.

        Positive angle_deg = right (CW).  Negative = left (CCW).
        No magnetometer used — reliable for short in-place turns.
        """
        start = self._odom_yaw_deg()
        if start is None:
            self.get_logger().warn('No odometry — cannot rotate')
            return
        # Odometry yaw increases CCW; right (CW) = decrease yaw
        target = start - angle_deg
        deadline = time.monotonic() + 15.0
        while rclpy.ok() and time.monotonic() < deadline:
            if self._cancel_requested:
                self._stop()
                return
            current = self._odom_yaw_deg()
            if current is None:
                time.sleep(0.05)
                continue
            err = angle_diff(target, current)   # degrees, -180..180
            if abs(err) <= 3.0:
                self._stop()
                return
            twist = Twist()
            # positive err → target is CCW of current → angular.z positive = CCW ✓
            twist.angular.z = max(-speed, min(speed, math.radians(err) * 2.0))
            self._cmd_pub.publish(twist)
            time.sleep(0.05)
        self._stop()

    def _drive_to_lidar_distance(self, target_dist_m, half_deg=25.0, timeout_s=15.0):
        """Drive forward until nearest LIDAR object in front sector is at target_dist_m."""
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            if self._cancel_requested:
                self._stop()
                return
            fwd = self._sector_min_range(0.0, half_deg)
            if fwd <= target_dist_m:
                self._stop()
                return
            twist = Twist()
            twist.linear.x = 0.3
            self._cmd_pub.publish(twist)
            time.sleep(0.1)
        self._stop()

    def _drive_forward_timed(self, dist_m, speed=0.3):
        """Drive forward for approximately dist_m (time-based)."""
        if dist_m <= 0:
            return
        deadline = time.monotonic() + dist_m / speed
        while rclpy.ok() and time.monotonic() < deadline:
            if self._cancel_requested:
                self._stop()
                return
            twist = Twist()
            twist.linear.x = speed
            self._cmd_pub.publish(twist)
            time.sleep(0.05)
        self._stop()

    def _scan_lidar_objects(self):
        """Cluster all current LIDAR returns into distinct objects.

        Returns a list of dicts sorted by distance (nearest first):
          'bearing_deg':  robot-frame bearing (0=forward, +ve=left, -ve=right)
          'distance_m':   range to cluster centre
          'world_bearing_deg': compass bearing to object
        """
        scan = self._scan
        if scan is None or not scan.ranges:
            return []

        result = self._heading_deg()
        heading = result[0] if result is not None else 0.0

        rmin = max(scan.range_min if scan.range_min > 0.0 else 0.05,
                   INSPECTION_MIN_OBJECT_DIST_M)
        rmax = min(scan.range_max if scan.range_max > 0.0 else 30.0,
                   INSPECTION_MAX_OBJECT_DIST_M)

        # Collect valid (angle_rad, range) points
        points = []
        a = scan.angle_min
        for r in scan.ranges:
            if not (math.isinf(r) or math.isnan(r)) and rmin <= r <= rmax:
                points.append((a, r))
            a += scan.angle_increment

        if not points:
            return []

        # Single-linkage cluster: group consecutive points within 0.4 rad (~23°)
        clusters_raw = []
        current = [points[0]]
        for pt in points[1:]:
            if abs(pt[0] - current[-1][0]) < 0.4:
                current.append(pt)
            else:
                clusters_raw.append(current)
                current = [pt]
        clusters_raw.append(current)

        objects = []
        for cl in clusters_raw:
            if len(cl) < 2:   # skip single-ray noise
                continue
            avg_angle = sum(p[0] for p in cl) / len(cl)
            avg_range = sum(p[1] for p in cl) / len(cl)
            wb = (heading - math.degrees(avg_angle) + 360) % 360
            objects.append({
                'bearing_deg': math.degrees(avg_angle),
                'distance_m': avg_range,
                'world_bearing_deg': wb,
            })

        objects.sort(key=lambda o: o['distance_m'])
        return objects

    def _scan_static_objects(self, duration_s=2.0, n_samples=8):
        """Return LIDAR objects that appear consistently across N scans (static filter).

        Objects seen in >= 60 % of snapshots are considered static.
        Each returned dict has 'bearing_deg', 'distance_m', 'world_bearing_deg'.
        """
        snapshots = []
        interval = duration_s / max(n_samples, 1)
        for _ in range(n_samples):
            time.sleep(interval)
            snapshots.append(self._scan_lidar_objects())

        if not snapshots:
            return []

        threshold = len(snapshots) * 0.6
        seen_buckets = set()
        stable = []

        # Build candidates from all snapshots sorted by distance
        all_objs = sorted(
            [o for snap in snapshots for o in snap],
            key=lambda o: o['distance_m'],
        )

        for candidate in all_objs:
            brg_bucket = round(candidate['bearing_deg'] / 15.0) * 15
            if brg_bucket in seen_buckets:
                continue

            matching = []
            for snap in snapshots:
                for obj in snap:
                    if (abs(angle_diff(obj['bearing_deg'], candidate['bearing_deg'])) < 15.0
                            and abs(obj['distance_m'] - candidate['distance_m']) < 0.5):
                        matching.append(obj)
                        break

            if len(matching) >= threshold:
                avg_brg  = sum(o['bearing_deg']       for o in matching) / len(matching)
                avg_dist = sum(o['distance_m']         for o in matching) / len(matching)
                avg_wb   = sum(o['world_bearing_deg']  for o in matching) / len(matching)
                stable.append({
                    'bearing_deg':       avg_brg,
                    'distance_m':        avg_dist,
                    'world_bearing_deg': avg_wb,
                })
                seen_buckets.add(brg_bucket)

        stable.sort(key=lambda o: o['distance_m'])
        return stable

    def _approach_and_photo(self, wp_idx):
        """After reaching waypoint: find marker (second-nearest static object),
        drive to it, burst-capture 10 photos, run majority-vote CV, log result.
        """
        label = f'[WP{wp_idx + 1}]'
        self._publish_status(f'{label} Scanning for objects...')
        time.sleep(0.5)

        objects = self._scan_static_objects(duration_s=1.5, n_samples=6)
        if not objects:
            objects = self._scan_lidar_objects()

        if not objects:
            self._publish_status(f'{label} No objects visible — skipping inspection')
            self._inspection_results.append({
                'waypoint': wp_idx + 1, 'cone_distance_m': 0.0,
                'cone_to_object_m': None, 'object_color': 'none',
                'object_shape': 'none', 'photos': [],
            })
            return

        cone = objects[0]  # nearest = waypoint cone

        # If a second object is visible use it as the marker; otherwise photograph the cone itself
        cone_to_marker_m = None
        if len(objects) >= 2:
            marker = objects[1]
            angle_between = math.radians(abs(angle_diff(marker['bearing_deg'], cone['bearing_deg'])))
            cone_to_marker_m = math.sqrt(
                cone['distance_m'] ** 2 + marker['distance_m'] ** 2
                - 2 * cone['distance_m'] * marker['distance_m'] * math.cos(angle_between))
            self._publish_status(
                f'{label} Cone {cone["distance_m"]:.2f}m | '
                f'Marker {marker["distance_m"]:.2f}m at {marker["bearing_deg"]:.1f}° | '
                f'cone→marker {cone_to_marker_m:.2f}m')
        else:
            marker = cone  # only one object — approach and photograph it
            self._publish_status(
                f'{label} Only 1 object at {cone["distance_m"]:.2f}m — approaching to photograph')

        # Turn toward marker
        self._rotate_by_angle_deg(-marker['bearing_deg'])
        if self._cancel_requested:
            return

        # Approach while capturing a photo every 0.5 s
        self._publish_status(f'{label} Approaching marker (capturing photos)...')
        try:
            import cv2
            from cv_bridge import CvBridge
            _bridge = CvBridge()
            _cv_ok = True
        except ImportError:
            _bridge = None
            _cv_ok = False

        os.makedirs(INSPECTION_PHOTO_DIR, exist_ok=True)
        photos = []
        last_photo_t = 0.0
        deadline = time.monotonic() + 20.0

        while rclpy.ok() and time.monotonic() < deadline:
            if self._cancel_requested:
                self._stop()
                return
            fwd = self._sector_min_range(0.0, 25.0)
            if fwd <= INSPECTION_APPROACH_STOP_M:
                self._stop()
                break
            twist = Twist()
            twist.linear.x = 0.3
            self._cmd_pub.publish(twist)
            now_t = time.monotonic()
            if _cv_ok and self._latest_image is not None and now_t - last_photo_t >= 0.5:
                try:
                    img = _bridge.imgmsg_to_cv2(self._latest_image, desired_encoding='bgr8')
                    path = os.path.join(INSPECTION_PHOTO_DIR,
                                        f'wp{wp_idx + 1}_{int(now_t * 1000):013d}.jpg')
                    cv2.imwrite(path, img)
                    photos.append(path)
                    last_photo_t = now_t
                except Exception:
                    pass
            time.sleep(0.1)
        else:
            self._stop()

        if self._cancel_requested:
            return

        # Burst-capture 10 more photos at rest for sharper shots
        self._publish_status(f'{label} Burst capturing 10 photos at stop...')
        photos += self._take_photos(wp_idx, 10)
        self._publish_status(f'{label} Total photos: {len(photos)} — running CV...')
        color, shape = self._classify_photos(photos, wp_idx)
        self._annotate_photos(photos, wp_idx, color, shape)
        self._publish_status(
            f'{label} Detected: {color} {shape}  '
            f'(from {len(photos)} photos, majority vote)')

        self._inspection_results.append({
            'waypoint':        wp_idx + 1,
            'cone_distance_m': cone['distance_m'],
            'cone_to_object_m': cone_to_marker_m,
            'object_color':    color,
            'object_shape':    shape,
            'photos':          photos,
        })
        self._publish_status(f'{label} Done!')

    def _cone_marker_inspect(self, wp_idx):
        """Cone side-step maneuver: turn right 90°, drive 1.2 m, turn left 90°.

        Then multi-scan static object detection to distinguish the cone (now to the
        left after the maneuver) from the marker object.  Computes cone→marker
        distance, logs it, and drives to the marker.
        """
        label = f'[WP{wp_idx + 1}]'
        self._publish_status(f'{label} Stopped at cone — pausing 3 s')
        time.sleep(3.0)
        if self._cancel_requested:
            return

        # ── 1. Side-step manoeuvre ────────────────────────────────────────────
        self._publish_status(f'{label} Turning right 90°')
        self._rotate_by_angle_deg(90)
        if self._cancel_requested:
            return

        self._publish_status(f'{label} Driving {INSPECTION_SIDE_OFFSET_M} m forward')
        self._drive_forward_timed(INSPECTION_SIDE_OFFSET_M, speed=0.25)
        if self._cancel_requested:
            return

        self._publish_status(f'{label} Turning left 90°')
        self._rotate_by_angle_deg(-90)
        if self._cancel_requested:
            return

        # ── 2. Multi-scan static object detection (full LIDAR FOV) ───────────
        self._publish_status(f'{label} Scanning for static objects (2 s)...')
        time.sleep(0.3)
        objects = self._scan_static_objects(duration_s=2.0, n_samples=8)

        if not objects:
            self._publish_status(f'{label} No static objects found — continuing')
            self._inspection_results.append({
                'waypoint': wp_idx + 1, 'cone_distance_m': 0.0,
                'cone_to_object_m': None, 'object_color': 'none',
                'object_shape': 'none', 'photos': [],
            })
            return

        # ── 3. Identify cone vs marker ────────────────────────────────────────
        # After the manoeuvre the original cone is ~39° to the LEFT of forward
        # and ~1.9 m away.  Pick the leftmost object within 4 m as the cone.
        left_cands = [o for o in objects if o['bearing_deg'] > 10.0 and o['distance_m'] < 4.0]
        cone = max(left_cands, key=lambda o: o['bearing_deg']) if left_cands else objects[0]

        others = [o for o in objects if o is not cone]
        marker = min(others, key=lambda o: o['distance_m']) if others else None

        self._publish_status(
            f'{label} Cone: {cone["bearing_deg"]:.1f}° bearing  {cone["distance_m"]:.2f} m')

        cone_to_marker_m = None
        if marker is None:
            self._publish_status(f'{label} No marker object found — only cone visible')
        else:
            # Law of cosines: distance between two polar points
            angle_between = math.radians(
                abs(angle_diff(marker['bearing_deg'], cone['bearing_deg'])))
            cone_to_marker_m = math.sqrt(
                cone['distance_m'] ** 2 + marker['distance_m'] ** 2
                - 2 * cone['distance_m'] * marker['distance_m'] * math.cos(angle_between))
            self._publish_status(
                f'{label} Marker: {marker["bearing_deg"]:.1f}° bearing  '
                f'{marker["distance_m"]:.2f} m | cone→marker {cone_to_marker_m:.2f} m')

        self._inspection_results.append({
            'waypoint':        wp_idx + 1,
            'cone_distance_m': cone['distance_m'],
            'cone_to_object_m': cone_to_marker_m,
            'object_color':    'unknown',
            'object_shape':    'unknown',
            'photos':          [],
        })

        if marker is None or self._cancel_requested:
            return

        # ── 4. Navigate to marker ─────────────────────────────────────────────
        # Turn toward marker using odometry (LIDAR bearing: positive=left, so negate
        # to convert to _rotate_by_angle_deg convention where positive=right)
        self._publish_status(
            f'{label} Turning toward marker ({marker["bearing_deg"]:.1f}° LIDAR)')
        self._rotate_by_angle_deg(-marker['bearing_deg'])
        if self._cancel_requested:
            return

        self._publish_status(f'{label} Approaching marker...')
        self._drive_to_lidar_distance(INSPECTION_APPROACH_STOP_M)
        self._publish_status(f'{label} Reached marker — inspection complete')

    def _take_photos(self, wp_idx, count):
        """Save `count` frames from /camera/rgb/image_raw. Returns list of saved paths."""
        try:
            import cv2
            from cv_bridge import CvBridge
        except ImportError:
            self._publish_status('cv2/cv_bridge unavailable — skipping photos')
            return []

        os.makedirs(INSPECTION_PHOTO_DIR, exist_ok=True)
        bridge = CvBridge()
        paths = []
        deadline = time.monotonic() + 15.0

        while len(paths) < count and rclpy.ok() and time.monotonic() < deadline:
            if self._latest_image is None:
                time.sleep(0.1)
                continue
            try:
                img = bridge.imgmsg_to_cv2(self._latest_image, desired_encoding='bgr8')
                ts = int(time.monotonic() * 1000)
                path = os.path.join(INSPECTION_PHOTO_DIR, f'wp{wp_idx + 1}_{ts:013d}.jpg')
                cv2.imwrite(path, img)
                paths.append(path)
                self._publish_status(f'Photo {len(paths)}/{count}: {path}')
            except Exception as e:
                self._publish_status(f'Photo error: {e}')
            time.sleep(0.5)
        return paths

    def _classify_photos(self, paths, wp_idx):
        """Run colour/shape detection over saved photos, return (color, shape) majority vote."""
        if not paths:
            return 'unknown', 'unknown'
        try:
            import cv2
        except ImportError:
            return 'unknown', 'unknown'

        votes = {}
        for path in paths:
            try:
                img = cv2.imread(path)
                if img is None:
                    continue
                result = _detect_col(img)
                if result is None:
                    continue
                color, _, shape = result
                key = (color, shape)
                votes[key] = votes.get(key, 0) + 1
            except Exception as e:
                self._publish_status(f'[WP{wp_idx + 1}] Detection error on {path}: {e}')

        if not votes:
            return 'unknown', 'unknown'
        best = max(votes, key=votes.__getitem__)
        return best  # (color, shape)

    def _annotate_photos(self, paths, wp_idx, color, shape):
        """Burn waypoint / detection label into every saved photo in-place."""
        try:
            import cv2
        except ImportError:
            return
        label = f'WP{wp_idx + 1} | {color} {shape}'
        font = cv2.FONT_HERSHEY_SIMPLEX
        for path in paths:
            try:
                img = cv2.imread(path)
                if img is None:
                    continue
                h, w = img.shape[:2]
                scale = max(0.6, w / 800.0)
                thickness = max(1, round(scale * 2))
                (tw, th), baseline = cv2.getTextSize(label, font, scale, thickness)
                # black backing rectangle so text is readable on any background
                cv2.rectangle(img, (8, 8), (14 + tw, 14 + th + baseline), (0, 0, 0), -1)
                cv2.putText(img, label, (10, 10 + th), font, scale,
                            (255, 255, 255), thickness, cv2.LINE_AA)
                cv2.imwrite(path, img)
            except Exception as e:
                self._publish_status(f'[WP{wp_idx + 1}] Annotate error {path}: {e}')

    def _write_summary(self):
        """Write a plain-text run summary to /data/run_summary_<timestamp>.txt."""
        os.makedirs(BAG_OUTPUT_DIR, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        path = os.path.join('/data', f'run_summary_{ts}.txt')
        lines = [
            '=== Navigation Run Summary ===',
            f'Date: {time.strftime("%Y-%m-%d %H:%M:%S")}',
            f'Waypoints completed: {len(self._inspection_results)} / {len(GPS_WAYPOINTS)}',
            '',
        ]
        for r in self._inspection_results:
            wp = r['waypoint']
            lines.append(f'Waypoint {wp}:')
            lines.append(f'  Cone distance from robot : {r["cone_distance_m"]:.2f} m')
            dist_str = f'{r["cone_to_object_m"]:.2f} m' if r['cone_to_object_m'] is not None else 'N/A'
            lines.append(f'  Cone → object distance   : {dist_str}')
            lines.append(f'  Object detected          : {r["object_color"]} {r["object_shape"]}')
            if r['photos']:
                lines.append('  Photos saved:')
                for p in r['photos']:
                    lines.append(f'    {p}')
            lines.append('')

        summary = '\n'.join(lines)
        try:
            with open(path, 'w') as f:
                f.write(summary)
            self._publish_status(f'Summary written → {path}')
        except Exception as e:
            self._publish_status(f'Summary write error: {e}')

    # ── Waypoint inspection state machine ─────────────────────────────────────

    def _waypoint_inspection(self, wp_idx):
        """After reaching waypoint wp_idx:
          1. LIDAR single-shot cluster scan → nearest object = cone, second = secondary
          2. Log cone→secondary distance
          3. Face the cone, approach to INSPECTION_CONE_APPROACH_M
          4. Turn 90° right, drive INSPECTION_SIDE_OFFSET_M → now beside the cone
          5. Face and approach the secondary object to INSPECTION_APPROACH_STOP_M
          6. Take INSPECTION_PHOTO_COUNT photos
        """
        self._publish_status(f'[WP{wp_idx + 1}] Inspection: scanning scene with LIDAR...')
        time.sleep(1.0)  # let sensors stabilise after stopping

        # ── 1. Single-shot cluster scan ───────────────────────────────────────
        objects = self._scan_lidar_objects()
        if len(objects) < 1:
            self._publish_status(f'[WP{wp_idx + 1}] No LIDAR objects found — skipping inspection')
            return

        cone = objects[0]  # nearest = cone marking the waypoint
        self._publish_status(
            f'[WP{wp_idx + 1}] Cone: bearing {cone["bearing_deg"]:.1f}°  '
            f'world {cone["world_bearing_deg"]:.1f}°  dist {cone["distance_m"]:.2f}m')

        secondary = objects[1] if len(objects) >= 2 else None
        cone_to_sec_m = None

        if secondary is None:
            self._publish_status(f'[WP{wp_idx + 1}] Only one object visible — will photograph cone')
        else:
            # Distance between cone and secondary via law of cosines
            cone_r = cone['distance_m']
            sec_r = secondary['distance_m']
            angle_between = math.radians(
                abs(angle_diff(secondary['bearing_deg'], cone['bearing_deg'])))
            cone_to_sec_m = math.sqrt(
                cone_r ** 2 + sec_r ** 2 - 2 * cone_r * sec_r * math.cos(angle_between))
            self._publish_status(
                f'[WP{wp_idx + 1}] Secondary: bearing {secondary["bearing_deg"]:.1f}°  '
                f'world {secondary["world_bearing_deg"]:.1f}°  '
                f'dist from robot {secondary["distance_m"]:.2f}m  '
                f'dist cone→secondary {cone_to_sec_m:.2f}m')

        # ── 2. Face the cone ──────────────────────────────────────────────────
        self._align_to_world_bearing(cone['world_bearing_deg'])
        if self._cancel_requested:
            return

        # ── 3. Approach the cone ──────────────────────────────────────────────
        self._publish_status(f'[WP{wp_idx + 1}] Approaching cone to {INSPECTION_CONE_APPROACH_M}m')
        self._drive_to_lidar_distance(INSPECTION_CONE_APPROACH_M)
        if self._cancel_requested:
            return

        # ── 4. Turn 90° right → cone is now to our left ───────────────────────
        self._publish_status(f'[WP{wp_idx + 1}] Turning right 90° to position beside cone')
        self._rotate_by_angle_deg(90)
        if self._cancel_requested:
            return

        self._drive_forward_timed(INSPECTION_SIDE_OFFSET_M)
        if self._cancel_requested:
            return

        # ── 5. Fresh scan from new position → face and approach secondary ────────
        # The world_bearing_deg from the initial scan is stale — robot has moved.
        # Re-scan: nearest cluster is the cone (now to our left), second is secondary.
        time.sleep(0.5)
        fresh = self._scan_lidar_objects()
        if len(fresh) >= 2:
            fresh_target = fresh[1]  # second nearest = secondary
        elif len(fresh) >= 1:
            fresh_target = fresh[0]
        else:
            fresh_target = None

        if fresh_target is None:
            self._publish_status(f'[WP{wp_idx + 1}] No target found after reposition — skipping approach')
        else:
            self._publish_status(
                f'[WP{wp_idx + 1}] Fresh scan: target at {fresh_target["world_bearing_deg"]:.1f}° '
                f'world, {fresh_target["distance_m"]:.2f}m away')
            self._align_to_world_bearing(fresh_target['world_bearing_deg'])
            if self._cancel_requested:
                return

            self._publish_status(f'[WP{wp_idx + 1}] Approaching target object...')
            self._drive_to_lidar_distance(INSPECTION_APPROACH_STOP_M)
        if self._cancel_requested:
            return

        # ── 6. Take photos and run colour/shape detection ─────────────────────
        self._publish_status(f'[WP{wp_idx + 1}] Taking {INSPECTION_PHOTO_COUNT} photos')
        photos = self._take_photos(wp_idx, INSPECTION_PHOTO_COUNT)

        detected_color, detected_shape = self._classify_photos(photos, wp_idx)
        self._annotate_photos(photos, wp_idx, detected_color, detected_shape)

        self._inspection_results.append({
            'waypoint':         wp_idx + 1,
            'cone_distance_m':  cone['distance_m'],
            'cone_to_object_m': cone_to_sec_m,
            'object_color':     detected_color,
            'object_shape':     detected_shape,
            'photos':           photos,
        })
        self._publish_status(
            f'[WP{wp_idx + 1}] Detected: {detected_color} {detected_shape}  '
            f'cone→object: {f"{cone_to_sec_m:.2f}m" if cone_to_sec_m is not None else "N/A"}')
        self._publish_status(f'[WP{wp_idx + 1}] Inspection complete!')

    def _sector_min_range(self, center_deg, half_deg):
        """Closest valid range in the laser sector [center-half, center+half].

        Angles are in the laser frame: 0° = forward, +° = CCW (left), -° = right.
        Returns inf if no scan or the sector is empty.
        """
        scan = self._scan
        if scan is None or not scan.ranges:
            return float('inf')
        center = math.radians(center_deg)
        half = math.radians(half_deg)
        rmin = scan.range_min if scan.range_min > 0.0 else 0.05
        rmax = scan.range_max if scan.range_max > 0.0 else 30.0
        min_r = float('inf')
        a = scan.angle_min
        inc = scan.angle_increment
        for r in scan.ranges:
            diff = (a - center + math.pi) % (2 * math.pi) - math.pi
            a += inc
            if abs(diff) > half or math.isinf(r) or math.isnan(r):
                continue
            if r < rmin or r > rmax:
                continue
            if r < min_r:
                min_r = r
        return min_r

    # ── Heading computation ───────────────────────────────────────────────────

    def _heading_deg(self, n_samples=20):
        """Return compass heading in degrees (0=N, 90=E), or None if not enough data.

        Use n_samples=3 for responsive in-place turn control; the default 20
        averages more samples for noise rejection during GPS navigation.
        """
        min_needed = min(10, n_samples)
        if len(self._mag_samples) < min_needed:
            return None
        recent = self._mag_samples[-n_samples:]
        avg_x = sum(s[0] for s in recent) / len(recent) - MAG_X_OFFSET
        avg_y = sum(s[1] for s in recent) / len(recent) - MAG_Y_OFFSET
        bearing = math.degrees(math.atan2(avg_x, avg_y))
        return (bearing + MAGNETIC_DECLINATION_DEG + 360) % 360, avg_x, avg_y

    # ── Always-on debug publishers ────────────────────────────────────────────

    def _publish_heading_debug(self):
        result = self._heading_deg()
        if result is None:
            return
        heading, avg_x, avg_y = result
        self._heading_pub.publish(Float32(data=float(heading)))
        self._mag_x_pub.publish(Float32(data=float(avg_x)))
        self._mag_y_pub.publish(Float32(data=float(avg_y)))

    def _publish_all_waypoints(self):
        """Publish all waypoints on separate topics so all pins show simultaneously."""
        stamp = self.get_clock().now().to_msg()
        for i, (lat, lon) in enumerate(GPS_WAYPOINTS):
            msg = NavSatFix()
            msg.header.stamp = stamp
            msg.header.frame_id = 'map'
            msg.latitude  = lat
            msg.longitude = lon
            msg.altitude  = 0.0
            msg.status.status = 0
            self._waypoint_pubs[i].publish(msg)

    # ── Navigation trigger ────────────────────────────────────────────────────

    # ── ROS bag recording ─────────────────────────────────────────────────────

    _BAG_TOPICS = [
        '/fix',
        '/imu/mag',
        '/scan',
        '/odom',
        '/camera/rgb/image_raw',
        '/camera/cone_detections',
        '/gps_nav/cmd_vel',
        '/gps_nav/status',
        '/gps_nav/heading_deg',
        '/gps_nav/bearing_deg',
        '/gps_nav/error_deg',
        '/gps_nav/distance_m',
        '/gps_nav/target_fix',
    ]

    def _start_bag(self):
        os.makedirs(BAG_OUTPUT_DIR, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        bag_path = os.path.join(BAG_OUTPUT_DIR, f'run_{ts}')
        cmd = ['ros2', 'bag', 'record', '-o', bag_path] + self._BAG_TOPICS
        self._bag_proc = subprocess.Popen(
            cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self._publish_status(f'Bag recording started → {bag_path}')

    def _stop_bag(self):
        proc = getattr(self, '_bag_proc', None)
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.kill()
            self._bag_proc = None
            self._publish_status('Bag recording stopped')

    def _trigger(self, msg):
        if not msg.data:
            if self._navigating:
                self._cancel_requested = True
                self.get_logger().info('Cancel requested — stopping navigation')
            return
        if self._navigating:
            return
        now = time.monotonic()
        if now - self._last_trigger_time < 5.0:
            return
        self._last_trigger_time = now
        self._cancel_requested = False
        threading.Thread(target=self._run, daemon=True).start()

    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _publish_status(self, text):
        self._status_pub.publish(String(data=text))
        self.get_logger().info(text)

    # ── Main navigation loop ──────────────────────────────────────────────────

    def _run(self):
        self._navigating = True
        self._active_pub.publish(Bool(data=True))
        try:
            self._publish_status('Waiting for GPS fix...')
            deadline = time.monotonic() + 30.0
            while self._gps_fix is None:
                time.sleep(0.2)
                if time.monotonic() > deadline:
                    self._publish_status('ERROR: No GPS fix after 30s.')
                    return

            origin_lat = self._gps_fix.latitude
            origin_lon = self._gps_fix.longitude
            self._publish_status(f'GPS origin: {origin_lat:.7f}, {origin_lon:.7f}')
            self._start_bag()

            for i, (wp_lat, wp_lon) in enumerate(GPS_WAYPOINTS):
                if self._cancel_requested:
                    break
                label = f'waypoint {i+1}'
                self._publish_status(f'Navigating to {label}: ({wp_lat:.7f}, {wp_lon:.7f})')
                self._publish_target_fix(wp_lat, wp_lon)
                self._drive_to(wp_lat, wp_lon)
                if self._cancel_requested:
                    self._publish_status('Navigation cancelled')
                    break
                self._publish_status(f'Reached {label}!')
                self._stop()
                self._approach_and_photo(i)
                if self._cancel_requested:
                    self._publish_status('Navigation cancelled during photo approach')
                    break
            else:
                # All waypoints reached — return to origin
                if not self._cancel_requested:
                    self._publish_status(f'All waypoints reached! Returning to origin: ({origin_lat:.7f}, {origin_lon:.7f})')
                    self._drive_to(origin_lat, origin_lon)
                    if not self._cancel_requested:
                        self._publish_status('Returned to origin! Navigation complete.')
                    else:
                        self._publish_status('Return to origin cancelled')
                else:
                    self._publish_status('Navigation cancelled')

        except Exception as e:
            self.get_logger().error(f'Navigation error: {e}')
            self._stop()
        finally:
            self._write_summary()
            self._inspection_results = []
            self._stop_bag()
            self._navigating = False
            self._active_pub.publish(Bool(data=False))

    def _publish_target_fix(self, lat, lon):
        """Publish target waypoint as NavSatFix so it appears on Foxglove Map."""
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.latitude  = lat
        msg.longitude = lon
        msg.altitude  = 0.0
        msg.status.status = 0
        self._target_fix_pub.publish(msg)

    def _front_object(self, half_deg=30.0):
        """Nearest LIDAR return in the front sector. Returns (bearing_rad, distance_m)."""
        scan = self._scan
        if scan is None or not scan.ranges:
            return 0.0, float('inf')
        half = math.radians(half_deg)
        rmin = scan.range_min if scan.range_min > 0.0 else 0.05
        rmax = scan.range_max if scan.range_max > 0.0 else 30.0
        best_r = float('inf')
        best_a = 0.0
        a = scan.angle_min
        for r in scan.ranges:
            diff = (a + math.pi) % (2 * math.pi) - math.pi
            if abs(diff) <= half and not (math.isinf(r) or math.isnan(r)) and rmin <= r <= rmax:
                if r < best_r:
                    best_r = r
                    best_a = diff
            a += scan.angle_increment
        return best_a, best_r

    def _drive_to(self, target_lat, target_lon):
        """Drive to a single waypoint. Runs in a background thread.

        State machine:
          drive  — PD-controller toward waypoint (GPS heading); obstacle → turn1
          turn1  — rotate 90° in AVOID_DIR
          arc    — quarter-circle (cone) or semicircle (obstacle) arc
          turn2  — rotate 90° back to original heading (full arc only)
          verify — wait 3s, check GPS is tight (<1.5m cluster), then declare done
        """
        last_log = 0.0
        state = 'drive'
        state_start = 0.0
        arc_radius = 0.0
        arc_angular = 0.0
        arc_duration = 0.0
        verify_start = 0.0
        cone_arc = False        # True → half arc (90°), skip turn2
        cur_avoid_dir = self._avoid_dir  # captured when avoidance triggers; alternates per run

        while rclpy.ok():
            if self._cancel_requested:
                self._stop()
                return
            time.sleep(0.1)

            self._publish_target_fix(target_lat, target_lon)

            if self._gps_fix is None:
                continue

            cur_lat = self._gps_fix.latitude
            cur_lon = self._gps_fix.longitude
            distance = haversine_distance(cur_lat, cur_lon, target_lat, target_lon)
            self._distance_pub.publish(Float32(data=float(distance)))

            # Transition to verification when very close
            if state == 'drive' and distance < GOAL_RADIUS_M:
                state = 'verify'
                verify_start = now_m
                self._gps_buffer.clear()  # Reset buffer — only use fresh reads during verify
                self._publish_status(f'Arrived! Verifying position for {WAYPOINT_VERIFY_S:.1f}s...')
                self._stop()

            result = self._heading_deg()
            if result is None:
                self.get_logger().warn('No magnetometer heading yet...')
                self._stop()
                continue
            robot_heading, _, _ = result

            goal_bearing = bearing_to(cur_lat, cur_lon, target_lat, target_lon)
            heading_error = angle_diff(goal_bearing, robot_heading)
            self._bearing_pub.publish(Float32(data=float(goal_bearing)))
            self._error_pub.publish(Float32(data=float(heading_error)))

            now_m = time.monotonic()
            fwd_brg_rad, fwd_dist = self._front_object(FRONT_CONE_HALF_DEG)

            twist = Twist()

            if state == 'drive':
                if distance < CONE_APPROACH_GPS_DIST_M:
                    # Near waypoint — no arc avoidance.
                    # Use a wide 90° half-angle scan so the cone can't slip past even if
                    # it's off-centre relative to the GPS waypoint coordinate.
                    wide_fwd = self._sector_min_range(0.0, 90.0)
                    if wide_fwd <= 1.0:
                        self._stop()
                        self._publish_status(
                            f'Object {wide_fwd:.2f}m ahead near waypoint — stopping')
                        return
                    error_rate_deg_per_s = 0.0
                    if self._last_heading_error_deg is not None and now_m - self._last_error_time > 0.01:
                        error_rate_deg_per_s = (heading_error - self._last_heading_error_deg) / (now_m - self._last_error_time)
                    self._last_heading_error_deg = heading_error
                    self._last_error_time = now_m
                    p_term = -math.radians(heading_error) * ANGULAR_GAIN
                    d_term = -math.radians(error_rate_deg_per_s) * ANGULAR_DAMPING
                    twist.linear.x  = LINEAR_SPEED
                    twist.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, p_term + d_term))
                    self._cmd_pub.publish(twist)
                elif fwd_dist < OBSTACLE_TRIGGER_DIST:
                    # Normal obstacle avoidance — full semicircle arc.
                    cur_avoid_dir = self._avoid_dir  # snapshot direction for this avoidance
                    arc_radius = ARC_RADIUS
                    arc_angular = ARC_LINEAR / arc_radius
                    arc_duration = math.pi * arc_radius / ARC_LINEAR
                    cone_arc = False
                    state = 'turn1'
                    state_start = now_m
                    side = 'RIGHT' if cur_avoid_dir < 0 else 'LEFT'
                    self._publish_status(
                        f'Obstacle {fwd_dist:.2f}m — 90° {side} then arc')
                    self._stop()
                else:
                    # PD controller toward waypoint + soft attraction toward nearest front object
                    error_rate_deg_per_s = 0.0
                    if self._last_heading_error_deg is not None and now_m - self._last_error_time > 0.01:
                        error_rate_deg_per_s = (heading_error - self._last_heading_error_deg) / (now_m - self._last_error_time)
                    self._last_heading_error_deg = heading_error
                    self._last_error_time = now_m
                    p_term = -math.radians(heading_error) * ANGULAR_GAIN
                    d_term = -math.radians(error_rate_deg_per_s) * ANGULAR_DAMPING
                    # Attract toward nearest static object in front sector (not active if too close)
                    attract = (fwd_brg_rad * ATTRACT_GAIN
                               if not math.isinf(fwd_dist) and fwd_dist > OBSTACLE_TRIGGER_DIST * 2
                               else 0.0)
                    twist.linear.x  = LINEAR_SPEED
                    twist.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, p_term + d_term + attract))
                    self._cmd_pub.publish(twist)

            elif state == 'turn1':
                elapsed = now_m - state_start
                if elapsed >= TURN_DURATION_S:
                    state = 'arc'
                    state_start = now_m
                    self._publish_status('Turn done → arcing around obstacle')
                else:
                    twist.angular.z = cur_avoid_dir * TURN_ANGULAR
                    self._cmd_pub.publish(twist)

            elif state == 'arc':
                elapsed = now_m - state_start
                if elapsed >= arc_duration:
                    if cone_arc:
                        # Half arc (90°) already restored original heading — skip turn2
                        cone_arc = False
                        state = 'drive'
                        self._publish_status('Cone half-arc done → resuming GPS nav')
                    else:
                        state = 'turn2'
                        state_start = now_m
                        self._publish_status('Arc done → turning to recover heading')
                elif fwd_dist < OBSTACLE_TRIGGER_DIST:
                    state = 'drive'
                    self._publish_status(
                        f'New obstacle at {fwd_dist:.2f}m during arc — stopping')
                    self._stop()
                else:
                    twist.linear.x  = ARC_LINEAR
                    twist.angular.z = -cur_avoid_dir * arc_angular
                    self._cmd_pub.publish(twist)

            elif state == 'turn2':
                elapsed = now_m - state_start
                if elapsed >= TURN_DURATION_S:
                    self._avoid_dir *= -1   # alternate side for next avoidance
                    state = 'drive'
                    next_side = 'LEFT' if self._avoid_dir > 0 else 'RIGHT'
                    self._publish_status(
                        f'Heading restored → resuming navigation (next avoid: {next_side})')
                else:
                    twist.angular.z = cur_avoid_dir * TURN_ANGULAR
                    self._cmd_pub.publish(twist)

            elif state == 'verify':
                # Wait for GPS to stabilize and verify tight cluster
                elapsed = now_m - verify_start
                gps_cluster = self._gps_cluster_radius()

                if elapsed >= WAYPOINT_VERIFY_S:
                    if gps_cluster < WAYPOINT_CLUSTER_M:
                        # GPS is tight — we're good
                        self._publish_status(
                            f'Position verified! GPS cluster={gps_cluster:.2f}m, '
                            f'dist={distance:.2f}m — waypoint reached')
                        return
                    else:
                        # GPS still bouncing — resume navigation
                        self._publish_status(
                            f'GPS still loose ({gps_cluster:.2f}m) — resuming navigation')
                        state = 'drive'
                else:
                    # Still verifying
                    self._publish_status(
                        f'Verifying... {elapsed:.1f}s (GPS cluster: {gps_cluster:.2f}m)')
                self._stop()

            if now_m - last_log >= 2.0:
                last_log = now_m
                self._publish_status(
                    f'[{state}] bearing={goal_bearing:.1f}°  heading={robot_heading:.1f}°  '
                    f'dist={distance:.2f}m  fwd={fwd_dist:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
