# Pioneer 3-AT Robot — Full Setup Guide From Scratch

## What this system does

The robot runs a full autonomous navigation stack:

```
Controller → gamepad_controller → cmd_vel → ariaNode → Pioneer motors
                                                ↓
                                         /odom + /imu
                                                ↓
                                          ekf_node (Kalman filter)
                                                ↓
                                      /odometry/filtered
                                                ↓
LIDAR (/scan) + odometry → slam_toolbox → /map + map→odom TF
                                                ↓
                              Nav2 (planner + controller) → cmd_vel
```

---

## System overview

| Component | Package | What it does |
|-----------|---------|-------------|
| `ariaNode` | p3at_robot | Talks to Pioneer motors via serial. Reads encoders → /odom. Reads gyro → /imu |
| `gamepad_controller` | p3at_robot | Reads PS4 controller, publishes cmd_vel in manual mode |
| `joy_node` | joy | Reads raw joystick hardware → /joy |
| `lakibeam1_scan_node` | lakibeam1 | Reads Lakibeam LIDAR over Ethernet → /scan |
| `static_transform_publisher` | tf2_ros | Publishes fixed transform: base_link → laser |
| `ekf_node` | robot_localization | Fuses /odom + /imu → /odometry/filtered (Kalman filter) |
| `slam_toolbox` | slam_toolbox | Uses /scan + odometry to build a map and localise |
| Nav2 stack | nav2_* | Plans paths and drives robot to goals autonomously |
| `foxglove_bridge` | foxglove_bridge | Streams all ROS2 data to browser for visualisation |

---

## Hardware connections

```
Laptop (WSL2)
    │  SSH / Docker build
    ▼
Robot NUC (192.168.2.211)
    ├── USB serial (/dev/ttyUSB0) ──→ Pioneer motor controller
    ├── USB (/dev/input/js0) ────────→ PS4-clone controller
    └── Ethernet (192.168.198.1) ───→ Lakibeam LIDAR (192.168.198.2)
```

---

## Part 1 — Build the Docker image (on your laptop)

### Why Docker?
The robot's NUC runs Ubuntu 24.04 with ROS2 Jazzy. Docker lets you build the entire software stack on your laptop and transfer it — no manual dependency installs on the robot.

### Prerequisites on laptop
```bash
# Install Docker
sudo apt install docker.io
sudo usermod -aG docker $USER   # add yourself to docker group (then log out/in)

# Install ROS2 Jazzy (for development only, not needed to run on robot)
# Follow: https://docs.ros.org/en/jazzy/Installation.html
```

### File structure
```
p3at_robot/
├── Dockerfile              ← builds the entire stack
├── CMakeLists.txt          ← builds ariaNode C++ code
├── package.xml             ← ROS2 package dependencies
├── launch/
│   └── gamepad.launch.py   ← starts everything with one command
├── config/
│   ├── slam_params.yaml    ← SLAM tuning parameters
│   ├── nav2_params.yaml    ← Nav2 tuning parameters
│   └── ekf.yaml            ← Kalman filter configuration
└── src/
    ├── ariaNode.cpp        ← C++ node: motors + odometry + IMU
    ├── gamepad_controller.py   ← manual driving
    ├── waypoint_controller.py  ← simple custom waypoint nav
    └── nav2_waypoints.py       ← sends goals to Nav2
```

### Build and transfer
```bash
cd ~/ros2_ws/src/p3at_robot

# Build the Docker image (~15-20 minutes first time)
sudo docker build -t pioneer_robot .

# Transfer to robot (~5-10 minutes depending on network)
sudo docker save pioneer_robot | gzip | ssh team6@192.168.2.211 "docker load"
```

---

## Part 2 — How the Dockerfile works

The Dockerfile does these steps in order:

1. **Base image**: `ros:jazzy-ros-base` — Ubuntu 24.04 + ROS2 Jazzy
2. **Install packages**: Nav2, SLAM toolbox, joy, robot_localization, foxglove, etc.
3. **Build AriaCoda**: clones from GitHub and compiles the Pioneer C++ library
4. **Install AriaCoda**: copies headers and .so library to /usr/local so it can be found at compile/runtime
5. **Build Lakibeam driver**: clones Lakibeam ROS2 driver and builds it
6. **Copy our code**: copies the p3at_robot package into the container
7. **Build our package**: runs colcon build to compile ariaNode

---

## Part 3 — Start the robot

### Every session checklist
1. Robot NUC is powered on
2. Pioneer motor controller is on (green light)
3. Lakibeam LIDAR is powered and Ethernet cable connected
4. PS4 controller plugged into robot USB

### SSH and start Docker
```bash
# On your laptop:
ssh team6@192.168.2.211
# password: Magic2010

# Start Docker container
docker run -it --rm --privileged --network host \
  -v /dev/input:/dev/input \
  pioneer_robot bash

# Inside Docker — set up LIDAR Ethernet and launch
ip addr add 192.168.198.1/24 dev enp89s0 2>/dev/null; ip link set enp89s0 up
source /ros2_ws/install/setup.bash
ros2 launch p3at_robot gamepad.launch.py
```

### What to look for in the output
```
[ariaNode-3] Connected to robot.              ← motors ready
[slam_toolbox] Registering sensor: [...]      ← SLAM running
[gamepad_controller] Controller connected!    ← gamepad ready
[lifecycle_manager] Managed nodes are active  ← Nav2 ready
```

### If something is missing
| Missing message | Problem | Fix |
|----------------|---------|-----|
| "Connected to robot" | /dev/ttyUSB0 not found | Check USB cable to motor controller |
| "Registering sensor" | SLAM not starting | Wait 30s, check /map topic |
| "Controller connected" | No gamepad | Check USB, run `ls /dev/input/js*` |
| Lakibeam not publishing | LIDAR Ethernet issue | Check `ip addr show enp89s0`, ping 192.168.198.2 |

---

## Part 4 — Driving manually

Hold **both triggers** + move **left stick** to drive.

| Button | Action |
|--------|--------|
| Circle (O) | Manual mode |
| X | Autonomous mode |
| Left stick up/down | Forward / backward |
| Left stick left/right | Turn |
| Both triggers | Dead-man switch (must hold in auto mode) |

---

## Part 5 — Building a map (SLAM)

SLAM (Simultaneous Localisation and Mapping) uses the LIDAR to build a map while tracking where the robot is.

1. Start the robot (Part 3)
2. Drive around the area you want to map — the robot needs to see all the walls/obstacles
3. Check the map is building:
```bash
# Second SSH terminal:
ssh team6@192.168.2.211
docker exec -it $(docker ps -q) bash
source /ros2_ws/install/setup.bash
ros2 topic hz /map    # should show ~0.2 Hz
```

4. Save the map when done:
```bash
ros2 run nav2_map_server map_saver_cli -f /root/my_map
```

5. Copy map out of Docker (so it survives container restart):
```bash
# On robot host (not inside Docker):
mkdir -p ~/maps
docker cp $(docker ps -q):/root/my_map.pgm ~/maps/my_map.pgm
docker cp $(docker ps -q):/root/my_map.yaml ~/maps/my_map.yaml
```

The map is saved as a `.pgm` image — white = free space, black = walls, grey = unknown.

---

## Part 6 — Visualising with Foxglove

Since RViz doesn't work over WSL2, use Foxglove Studio instead.

1. Open **studio.foxglove.dev** in your browser (or download the desktop app)
2. Click **Open connection** → **Rosbridge WebSocket**
3. URL: `ws://192.168.2.211:8765`
4. Add panels:
   - **Map** → subscribe to `/map`
   - **3D** → add `/scan` (LaserScan), `/tf` (TF tree)
   - **Plot** → subscribe to `/odom` for position data
   - **Topic list** → see all active topics

---

## Part 7 — Autonomous navigation with Nav2

### How it works
1. **Global planner** (NavFn): looks at the full map, calculates a path from current position to goal
2. **Local controller** (DWB): takes the path, generates smooth velocity commands in real time
3. **Costmaps**: inflate obstacles on the map so the robot keeps a safe distance from walls
4. **Recovery behaviours**: if the robot gets stuck, it spins in place or backs up

### Run autonomous waypoints
```bash
# Second terminal (while gamepad.launch.py is running):
docker exec -it $(docker ps -q) bash
source /ros2_ws/install/setup.bash
ros2 run p3at_robot nav2_waypoints.py
```

### Edit the waypoints
Open `src/nav2_waypoints.py` and change the coordinates:
```python
waypoints = [
    make_pose(nav, 2.0, 0.0,   0.0),   # (x metres, y metres, heading degrees)
    make_pose(nav, 2.0, 2.0,  90.0),
    make_pose(nav, 0.0, 2.0, 180.0),
    make_pose(nav, 0.0, 0.0,   0.0),
]
```
Coordinates are relative to where the robot started (map origin = robot start position).

After editing, rebuild and deploy (Part 1 — Build section).

---

## Part 8 — The Kalman Filter (EKF)

The `ekf_node` from `robot_localization` fuses two sensor sources:

| Sensor | Topic | What it provides |
|--------|-------|-----------------|
| Wheel encoders | `/odom` | Position (x,y), heading, velocity |
| Pioneer gyro | `/imu/data_raw` | Rotation rate (angular_velocity.z) |

**Output**: `/odometry/filtered` — a smoother, more accurate pose estimate

**Why bother?** Wheels can slip, especially when turning. The gyro measures actual rotation independently, so when wheels slip the EKF corrects the heading estimate.

### Calibration
The covariance values in `config/ekf.yaml` are the calibration:
- Smaller number = trust this sensor more
- Larger number = trust this sensor less

To check if calibration is needed:
```bash
# With robot stationary, check gyro bias:
ros2 topic echo /imu/data_raw | grep -A3 angular_velocity
# angular_velocity.z should be ~0.0 when not moving
# If it's consistently e.g. 0.02, that's the bias
```

If there's a consistent bias, adjust `angular_velocity_covariance[8]` in `ariaNode.cpp` to a larger value (tells EKF to trust the gyro less).

---

## Part 9 — Rebuilding after code changes

Any time you edit a file in `p3at_robot/`, push from your laptop then rebuild on the robot:

```bash
# On your laptop:
git push

# SSH into robot and rebuild:
ssh team6@192.168.2.211
cd ~/ros2_ws/src/p3at_robot
git pull
docker build -t pioneer_robot .
```

Then restart the container.

---

## Troubleshooting quick reference

```bash
# Check all nodes are running
ros2 node list

# Check all topics
ros2 topic list

# Check a topic is publishing
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /map

# Check TF tree
ros2 run tf2_tools view_frames

# Check SLAM state
ros2 lifecycle get /slam_toolbox

# Check Nav2 state  
ros2 lifecycle get /bt_navigator

# Force stop container
docker stop $(docker ps -q)

# Check if controller is detected
ls /dev/input/js*
```