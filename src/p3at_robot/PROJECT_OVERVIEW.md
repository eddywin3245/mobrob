# P3-AT Robot — Project Overview

Autonomous Pioneer P3-AT robot that drives a GPS waypoint route marked by orange
traffic cones, weaves through cones between waypoints 1 and 2, takes a photo at
each marker (keeping it on the robot's right), identifies a nearby coloured object
(shape + distance), and returns to its start. LIDAR handles collision avoidance.
A gamepad with a deadman switch can disable or override autonomy at any time.

This document explains the runtime: every node, every topic, who talks to whom,
which algorithms run where, and the launch wiring.

---

## 1. Hardware

| Sensor / Actuator | Interface | Role |
|---|---|---|
| Pioneer P3-AT base | `/dev/ttyUSB0` via AriaCoda (libAria.so) | Wheel drive + wheel odometry |
| Lakibeam RichBeam LIDAR | Ethernet 192.168.198.2:2368 | 2-D scan @ 30 Hz (270° FOV, z = 0.30 m) |
| OAK-D-S2 camera | USB, DepthAI 3.5 | RGB 320×240 for detection, stereo depth pipeline (no IMU — S2 variant lacks one) |
| Phidget IMU + magnetometer | USB | Backup orientation + compass |
| GPS (NMEA) | `/dev/ttyACM0` @ 9600 baud | `/fix` NavSatFix, ~1 Hz |
| PS4 gamepad | Bluetooth (joy_node) | Manual drive, mode select, dead-man |

Static TF:
- `base_link → laser`  (0.20, 0, 0.30)
- `base_link → camera_rgb_optical_frame`  (0.15, 0, 0.10)
- `base_link → camera_depth_optical_frame` (0.15, 0, 0.10)
- `odom ← base_link`  published by EKF
- `map ← odom`  published by slam_toolbox

---

## 2. Node inventory

Every `ros2 run`-able node that runs in the full system.

### Robot base / sensors

| Node | Source | Purpose |
|---|---|---|
| `aria_node` | [src/ariaNode.cpp](src/ariaNode.cpp) | Pioneer driver — drives wheels, publishes `/odom`, TF `odom→base_link` |
| `joy_node` | `joy` pkg | Reads gamepad → `/joy` |
| `gamepad_controller` | [src/gamepad_controller.py](src/gamepad_controller.py) | Mode state machine, deadman gating, manual drive, assist orbit |
| `phidget_imu` | [src/phidget_imu.py](src/phidget_imu.py) | Publishes `/imu/data`, `/imu/mag` from Phidget |
| `gps_node` | `nmea_navsat_driver` | Publishes `/fix` from NMEA |
| `richbeam_lidar_node0` | `lakibeam1` | Publishes `/scan` |
| `oak_camera` | [src/oak_camera.py](src/oak_camera.py) | Publishes `/camera/rgb/image_raw` (320×240 BGR) |
| `ekf_node` | `robot_localization` | Fuses wheel odom + IMU → publishes `odom → base_link` TF |
| `slam_toolbox` | `slam_toolbox` | Async SLAM on LIDAR — publishes `/map` and `map → odom` TF |

### Autonomy / perception

| Node | Source | Purpose |
|---|---|---|
| `gps_waypoint_navigator` | [src/gps_waypoint_navigator.py](src/gps_waypoint_navigator.py) | Main autonomy: GPS nav, cone weave, right-side pass, photo capture, obstacle avoidance, journey summary |
| `cone_detection` | [src/cone_detection.py](src/cone_detection.py) | Orange-cone detection from RGB → publishes bounding boxes + distance estimates |
| `waypoint_scanner` | [src/waypoint_scanner.py](src/waypoint_scanner.py) | 360° scan at each waypoint to identify coloured object (shape + colour + distance) |

### Visualization

| Node | Purpose |
|---|---|
| `foxglove_bridge` | Publishes everything on `ws://*:8765` for Foxglove Studio |

### Nav2 stack (launched but not currently on the critical path)

Nav2 is wired up via [launch/gamepad.launch.py](launch/gamepad.launch.py) and its
config [config/nav2_params.yaml](config/nav2_params.yaml). `controller_server`,
`planner_server`, `behavior_server`, `bt_navigator`, `waypoint_follower` all come
up via `nav2_lifecycle_manager`. Our custom GPS navigator does not use Nav2 —
its `/cmd_vel` is generated inline with simple haversine+bearing+P-control. Nav2
remains available for future nav-stack-based routes.

---

## 3. Topics — who publishes, who subscribes

### Control & mode

| Topic | Type | Publishers | Subscribers | Meaning |
|---|---|---|---|---|
| `/joy` | sensor_msgs/Joy | joy_node | gamepad_controller | Raw gamepad state |
| `/cmd_vel` | geometry_msgs/Twist | gamepad_controller | aria_node | Final velocity command (gated by deadman) |
| `/gps_nav/cmd_vel` | Twist | gps_waypoint_navigator | gamepad_controller | Autonomy's proposed velocity — forwarded only if deadman held + auto mode |
| `/nav2/cmd_vel` | Twist | nav2 controller/behaviors | gamepad_controller | Nav2's velocity (same gating) |
| `/start_gps_nav` | std_msgs/Bool | gamepad_controller | gps_waypoint_navigator | True = start navigation, False = cancel |
| `/robot_mode` | std_msgs/String | gamepad_controller | (debug) | Current mode: `manual` / `auto` |

### Sensor raw topics

| Topic | Type | Publisher | Subscribers |
|---|---|---|---|
| `/odom` | nav_msgs/Odometry | aria_node | ekf_node |
| `/scan` | sensor_msgs/LaserScan | lakibeam | slam_toolbox, gps_waypoint_navigator, gamepad_controller, waypoint_scanner |
| `/fix` | sensor_msgs/NavSatFix | gps_node | gps_waypoint_navigator |
| `/imu/data`, `/imu/mag` | Imu, MagneticField | phidget_imu | ekf_node, gps_waypoint_navigator |
| `/camera/rgb/image_raw` | sensor_msgs/Image | oak_camera | cone_detection, waypoint_scanner, gps_waypoint_navigator |

### Perception

| Topic | Type | Publisher | Subscribers | Meaning |
|---|---|---|---|---|
| `/camera/cone_detections` | std_msgs/String (JSON) | cone_detection | gps_waypoint_navigator | `{detected, count, side, cones: [{x,y,w,h,distance_m}, …]}` |
| `/camera/detected_cones` | sensor_msgs/Image | cone_detection | Foxglove | Annotated frame with bboxes + distance labels |
| `/camera/cone_markers` | visualization_msgs/MarkerArray | cone_detection | Foxglove | 3-D text markers positioned by pixel+distance |

### Waypoint-scanner handshake

| Topic | Type | Publisher | Subscriber | Meaning |
|---|---|---|---|---|
| `/waypoint_scan/start` | String | gps_waypoint_navigator | waypoint_scanner | Waypoint label, starts scanning |
| `/waypoint_scan/complete` | Bool | waypoint_scanner | gps_waypoint_navigator | Scan finished, result published |
| `/waypoint_scan/object_info` | String (JSON) | waypoint_scanner | gps_waypoint_navigator | `{label, detected, shape, colour, bearing_deg, distance_m, area_px}` |
| `/waypoint_scan/object_image` | Image | waypoint_scanner | Foxglove | Annotated frame of best detection |

### Navigator debug topics (all `/gps_nav/…`)

All Float32 unless noted. Always live for Foxglove:

| Topic | Type | Meaning |
|---|---|---|
| `/gps_nav/heading_deg` | Float32 | Compass heading from Phidget magnetometer |
| `/gps_nav/mag_x_raw`, `/mag_y_raw` | Float32 | Hard-iron-corrected mag components |
| `/gps_nav/bearing_deg` | Float32 | Great-circle bearing to current goal |
| `/gps_nav/error_deg` | Float32 | `bearing − heading` wrapped ±180° |
| `/gps_nav/distance_m` | Float32 | Haversine distance to current goal |
| `/gps_nav/target_fix` | NavSatFix | Current goal pin for Foxglove Map |
| `/gps_nav/current_fix` | NavSatFix | Median-filtered robot fix for map |
| `/gps_nav/waypoint_N` | NavSatFix | One topic per configured waypoint, always published |
| `/gps_nav/active` | Bool | True while nav thread running |
| `/gps_nav/status` | String | Human-readable status log |

---

## 4. Control flow

### Gamepad → motion (safety layer)

```
joy_node ──/joy──► gamepad_controller ──/cmd_vel──► aria_node ──► wheels
                       │
                       ├── mode:  X = auto, O = manual
                       ├── R2    = dead-man (release → twist = 0)
                       ├── SQUARE (auto + no assist) → /start_gps_nav
                       └── TRIANGLE (auto) → toggle cone-orbit assist
```

In **manual** mode the left/right sticks drive directly.
In **auto** mode, external nodes (`gps_nav/cmd_vel`, `nav2/cmd_vel`) are forwarded
to `/cmd_vel` **only if** the deadman is held. Cone-orbit assist is an inline
behavior where the joystick drives but the controller auto-orbits any detected
cone at 0.5 m.

### GPS navigation state machine ([gps_waypoint_navigator.py](src/gps_waypoint_navigator.py))

```
idle ── Square press ─► _run()
                         │
                         ▼
                   capture origin GPS
                   append origin as final waypoint (return-to-start)
                         │
                         ▼
              ┌──► for each waypoint:
              │      _drive_to(wp)          (with 1 m left-perp offset for marker pass)
              │        │  loop @ 10 Hz:
              │        │    compute dist, bearing, heading_error
              │        │    if cones detected AND not near marker → weave mode
              │        │    if LIDAR < 2.5 m ahead           → wall-follow (DistBug)
              │        │    else                              → go-to-goal with approach slowdown
              │        │    if dist < GOAL_RADIUS_M → done
              │        │
              │      save marker photo
              │      _scan_at_waypoint()     (360° rotation, waypoint_scanner detects object)
              │      append entry to journey log
              │      pause 3 s
              └──◄ next waypoint
              on completion: _print_journey_summary() → file + log
```

---

## 5. Algorithms — where each one lives

### Heading ([gps_waypoint_navigator.py `_heading_deg`](src/gps_waypoint_navigator.py))

Phidget magnetometer. Averages last 20 samples, subtracts hard-iron offsets
(`MAG_X_OFFSET`, `MAG_Y_OFFSET` from `mag_calibrate.py`), `atan2(x, y)` →
bearing, add magnetic declination (Perth = 1.4°). Note: the OAK-D-S2 camera
has no onboard IMU, so magnetometer is the only compass source.

### GPS filtering ([gps_waypoint_navigator.py `_gps_cb`](src/gps_waypoint_navigator.py))

Every `/fix`:
1. Reject fixes that jump >10 m from the last accepted fix (multipath / glitches).
2. Keep a rolling window of 5 fixes; return the **median lat** and **median lon**.
3. Publish filtered fix on `/gps_nav/current_fix`.

Kills single-sample noise without lag that would hurt a moving robot.

### Haversine + great-circle bearing

Standard formulas, `haversine_distance()` and `bearing_to()`. Used everywhere for
distance-to-goal and desired heading.

### Go-to-goal controller

```
twist.linear.x  = LINEAR_SPEED, ramped from 0.30 → 0.12 m/s within APPROACH_DIST_M (3 m)
twist.angular.z = −radians(heading_error) × ANGULAR_GAIN, clamped ±0.8 rad/s
```

Angular gain halved when weaving to prevent zig-zag.

### Right-side-pass (spec #2)

At the start of each `_drive_to()` for a marker waypoint:
1. Compute bearing from robot to the true waypoint.
2. Offset the *target* 1 m perpendicular-LEFT of the cone.
3. Drive to the offset. Since the robot arrives and departs left of the cone,
   the cone naturally lies on its **right**.

Suppressed for the final return-to-start leg (no marker there).

### DistBug wall-follow ([gps_waypoint_navigator.py](src/gps_waypoint_navigator.py))

Triggers when forward cone (±30°) sees something < 2.5 m:
- Latches a side (alternates on each new trigger, holds during a 4 s lockout).
- Tracks wall using the closer of the 90° side sector or the 45° corner sector
  with a P-controller targeting 1.5 m side-distance.
- Front-blocking boost: extra turn-away added when front < 1.8 m.
- Leaves wall-follow when: (a) wall lost for > 1.2 s, **or** (b) committed ≥ 1.5 s
  AND goal cone clear > 2.2 m, AND closed ≥ 0.5 m on the hit distance.

### Orange-cone detection ([cone_detection.py](src/cone_detection.py))

Per frame @ 5 Hz:
- Custom BGR mask (not HSV): `r > 130 ∧ g < 80 ∧ b < 100 ∧ (r−g) > 70 ∧ (r−b) > 70`.
- Morphological close + open.
- For each contour: area > 800 AND aspect (h/w) > 1.2 → accept.
- Distance estimate from bbox area (calibrated constant).
- Publishes JSON + annotated image + 3-D text markers.

### Pixel → GPS transform (used by weave) ([gps_waypoint_navigator.py `_pixel_to_gps_coordinate`](src/gps_waypoint_navigator.py))

1. Pixel → normalized camera coords using 50° HFOV assumption.
2. 3-D point in `camera_rgb_optical_frame` using the cone's estimated distance.
3. Static `camera → base_link` offset (+0.15 m, +0.10 m).
4. TF lookup `map ← base_link` from SLAM, apply yaw rotation.
5. Convert map-frame metres → GPS offset via inverse haversine around current fix.

### Cone weaving ([gps_waypoint_navigator.py `_generate_weave_waypoints`](src/gps_waypoint_navigator.py))

- Convert each detected cone to GPS coords (above).
- Sort by distance, closest first.
- For each cone, generate a ±0.3 m perpendicular micro-waypoint (alternating
  left / right by index).
- Append main goal at the end.
- Navigator progresses through micro-waypoints one at a time; when all cones
  have cleared, resumes direct nav.
- **Auto-suppressed** within 5 m of the current waypoint (the cone there *is*
  the marker, we don't weave around it, we pass it on the right via offset).

### 360° waypoint scan ([waypoint_scanner.py](src/waypoint_scanner.py))

Triggered by `/waypoint_scan/start`. For ~20 s:
- HSV mask: keep pixels that are **not grass** AND **not orange**, with enough
  saturation/value to exclude sky/shadow.
- Morphological close + open, find largest contour.
- Bearing from pixel centre × camera HFOV.
- Distance: currently LIDAR range at that bearing (±3° cone).
  **Planned upgrade**: OAK depth median inside the bbox.
- **Shape classifier:** circularity `4π·area/perim²`; if > 0.8 → `circle`.
  Else `cv2.approxPolyDP` vertex count → triangle / square / rectangle /
  pentagon / hexagon / polygon(N).
- **Colour classifier:** dominant HSV hue inside the contour → red / orange /
  yellow / green / blue / purple / pink / grey.
- Keeps the largest detection across the sweep, publishes annotated image +
  `object_info` JSON + `complete` flag.

### Obstacle avoidance (gamepad-side emergency)

[gamepad_controller.py](src/gamepad_controller.py) also watches `/scan` and can
brake independently of the navigator. Belt-and-braces safety.

---

## 6. Launch architecture

### Main launch — [launch/gamepad.launch.py](launch/gamepad.launch.py)

`ros2 launch p3at_robot gamepad.launch.py` brings up the full system:

1. `ip addr add 192.168.198.1/24 dev enp89s0` — LIDAR Ethernet
2. `joy_node`, `gamepad_controller`, `aria_node`, `lakibeam` LIDAR
3. Static TF publishers: `laser_tf`, `camera_rgb_tf`, `camera_depth_tf`
4. `ekf_node` (reads [config/ekf.yaml](config/ekf.yaml))
5. `slam_toolbox` + `lifecycle_manager_slam` (reads [config/slam_params.yaml](config/slam_params.yaml))
6. Nav2 stack: controller/planner/behavior/bt_navigator/waypoint_follower
   (reads [config/nav2_params.yaml](config/nav2_params.yaml)) + `lifecycle_manager_navigation`
7. `phidget_imu`, `gps_node`, `gps_waypoint_navigator`
8. `oak_camera`, `cone_detection`, `waypoint_scanner`
9. `foxglove_bridge` on :8765

### Isolation launches

- [launch/lidar_test.launch.py](launch/lidar_test.launch.py) — LIDAR only
- [launch/oak_depth_test.launch.py](launch/oak_depth_test.launch.py) — OAK-D depth test only

---

## 7. Configuration files

| File | Purpose |
|---|---|
| [config/ekf.yaml](config/ekf.yaml) | robot_localization EKF — fuses `/odom` + `/imu/data` → `odom→base_link` TF |
| [config/slam_params.yaml](config/slam_params.yaml) | slam_toolbox async params — scan_buffer_size=100 (raised to prevent overflow) |
| [config/nav2_params.yaml](config/nav2_params.yaml) | Nav2 controller / planner / behavior params |
| [config/teleop.yaml](config/teleop.yaml) | Joystick → teleop mapping (legacy, not on current path) |

### Key navigator tuning constants ([gps_waypoint_navigator.py:33-90](src/gps_waypoint_navigator.py))

| Constant | Value | Meaning |
|---|---|---|
| `GPS_WAYPOINTS` | list | Waypoint lat/lon, edited before each run |
| `MAG_X_OFFSET`, `MAG_Y_OFFSET` | calibrated | Hard-iron correction from `mag_calibrate.py` |
| `MAGNETIC_DECLINATION_DEG` | 1.4 | Perth declination |
| `GPS_FILTER_WINDOW` | 5 | Median-filter window |
| `GPS_JUMP_REJECT_M` | 10 | Reject fix if it jumps > this distance |
| `GOAL_RADIUS_M` | 1.5 | Waypoint reached threshold (spec: 1-2 m) |
| `APPROACH_DIST_M` | 3.0 | Start slowdown within this distance |
| `APPROACH_MIN_SPEED` | 0.12 m/s | Floor on linear speed near goal |
| `LINEAR_SPEED` | 0.30 m/s | Cruise forward speed |
| `ANGULAR_GAIN` | 1.2 | P-gain on heading error |
| `MAX_ANGULAR` | 0.8 rad/s | Turn-rate clamp |
| `MARKER_PASS_OFFSET_M` | 1.0 | Left-perpendicular offset for right-side pass |
| `MARKER_PROXIMITY_M` | 5.0 | Within this, disable weaving (cone is the marker) |
| `OBSTACLE_TRIGGER_DIST` | 2.5 m | Forward LIDAR → enter wall-follow |
| `OBSTACLE_KEEP_DIST` | 1.5 m | Target side-distance while wall-following |

---

## 8. Output files (per run)

All under `~/waypoint_photos/`:

- `waypoint_N_marker_<timestamp>.jpg` — photo of the marker cone on arrival
- `waypoint_N_object_<timestamp>.jpg` — photo of the best coloured-object detection
- `journey_summary_<timestamp>.txt` — human-readable per-waypoint report + raw JSON

The journey summary includes: waypoint GPS, marker photo path, object shape +
colour + distance + bearing, object photo path.

---

## 9. Spec coverage

| Spec item | Status | Implementation |
|---|---|---|
| 1. GPS waypoints + return to start | ✓ | Origin captured on `_run()`, appended as final waypoint |
| 2. Cone-marked waypoints (1-2 m arrival, photo, cone on right) | ✓ | `GOAL_RADIUS_M=1.5`, `_save_photo`, 1 m left-perp approach |
| 3. Weave through cones between WP1 and WP2 | ✓ | Pixel→GPS transform, alternating ±0.3 m micro-waypoints |
| 4. Coloured object: shape, photo, distance from marker | ⚠ | Shape + colour + photo done; distance is currently LIDAR range **from robot** (≈ from marker, since robot is within 1-2 m). Depth-based upgrade pending |
| 5. Journey summary on completion | ✓ | `_print_journey_summary` → log + file |
| 6. LIDAR collision avoidance | ✓ | DistBug wall-follow in navigator + emergency brake in gamepad |
| 7a. Auto mode + deadman | ✓ | X button, R2 deadman, release → stop |
| 7b. Manual drive | ✓ | O button, left stick drives |

---

## 10. Known tuning / pending items

1. Magnetometer hard-iron offsets — re-run `mag_calibrate.py` on the field if heading drifts.
2. Cone detection BGR thresholds — may need outdoor-light adjustment.
3. Waypoint-scanner HSV — grass + orange ranges for the actual field ([waypoint_scanner.py:32](src/waypoint_scanner.py#L32)).
4. Camera HFOV — `CAMERA_HFOV_DEG = 66.0` in scanner; measure the real OAK HFOV.
5. **Pending:** add depth stream to `oak_camera.py`, swap scanner's distance
   source from LIDAR → depth (handles short/tall objects LIDAR misses).
6. **Pending:** cone-pos/object-pos subtraction for spec-accurate "distance from marker".
7. **Pending:** standalone bucket/helmet detector (we have a standalone tester
   script but have not wired it into the scanner yet).

---

## 11. File index (quick reference)

### Source
- [src/ariaNode.cpp](src/ariaNode.cpp) — Pioneer driver (C++)
- [src/gamepad_controller.py](src/gamepad_controller.py) — mode/deadman/assist
- [src/gps_waypoint_navigator.py](src/gps_waypoint_navigator.py) — main autonomy
- [src/cone_detection.py](src/cone_detection.py) — orange cones from RGB
- [src/waypoint_scanner.py](src/waypoint_scanner.py) — 360° object scan
- [src/oak_camera.py](src/oak_camera.py) — OAK-D RGB publisher
- [src/phidget_imu.py](src/phidget_imu.py) — Phidget IMU publisher
- [src/mag_calibrate.py](src/mag_calibrate.py) — hard-iron calibration helper
- [src/nav2_waypoints.py](src/nav2_waypoints.py) — Nav2 waypoint follower (unused on critical path)

### Tools (not in main launch)
- [src/test_oak_depth.py](src/test_oak_depth.py) — standalone OAK depth test
- [src/test_oak_device.py](src/test_oak_device.py) — OAK USB diagnostic
- [src/capture_rgbd.py](src/capture_rgbd.py) — RGBD dataset capture
- [src/test_spatial_detection.py](src/test_spatial_detection.py) — YOLO spatial-detection experiment

### Launch
- [launch/gamepad.launch.py](launch/gamepad.launch.py) — full system
- [launch/lidar_test.launch.py](launch/lidar_test.launch.py) — LIDAR only
- [launch/oak_depth_test.launch.py](launch/oak_depth_test.launch.py) — OAK depth only

### Config
- [config/ekf.yaml](config/ekf.yaml)
- [config/slam_params.yaml](config/slam_params.yaml)
- [config/nav2_params.yaml](config/nav2_params.yaml)
- [config/teleop.yaml](config/teleop.yaml)
