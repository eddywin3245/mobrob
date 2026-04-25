# Pioneer Robot Quick Start Guide

## 1. SSH into the Robot
```bash
ssh team6@192.168.2.113 # can change, if not connecting, run ipconfig on robot to find ip 
# password: Magic2010

 ssh team6@10.106.4.252
 ssh team6@pioneer6-NUC11PHi7

```

---
#compile docker 

cd ~/ros2_ws/src/p3at_robot && git fetch origin && git checkout work-from-old && git pull origin work-from-old
git branch --show-current   # should print: work-from-old
docker build -t pioneer_robot .

## 2. Start the Robot (every session)

**Step 1 — Start Docker container:**
```bash
cd ~/ros2_ws/src/p3at_robot && git fetch origin && git checkout j && git pull origin j
docker build -t pioneer_robot .

docker run -it --rm --privileged --network host \
  -v /dev/input:/dev/input \
  -v /dev/bus/usb:/dev/bus/usb \
  -v ~/robot_data:/data \
  -e ROS_DOMAIN_ID=6 \
  pioneer_robot 

> **Important:** `ROS_DOMAIN_ID=6` isolates this robot's ROS2 network from other teams on the same WiFi. Every team must use a different number.

**Step 2 — Inside Docker, launch everything:**
```bash
 source /ros2_ws/install/setup.bash
colcon build --packages-select p3at_robot && source install/setup.bash
ros2 launch p3at_robot gamepad.launch.py
 
```


python3 -m http.server 8080 --directory ~/robot_data


http://192.168.2.113:8080



> The LIDAR Ethernet interface (`192.168.198.1/24`) is configured automatically by the launch file.

Wait for these messages before using the robot:
- `Connected to robot.` — motors ready
- `Registering sensor: [Custom Described Lidar]` — SLAM running
- `Controller connected!` — gamepad ready

---

## 3. Controller Buttons

| Button | Action |
|--------|--------|
| Circle (O) | Manual driving mode |
| X | Autonomous waypoint mode |
| Left stick | Drive (up/down = forward/back, left/right = turn) |
| Both triggers held | Dead-man switch (required in auto mode) |
| Triangle (auto + deadman) | Spin 90 degrees in place |

---

## 4. Save a Map

After driving around to build the map, open a **second SSH terminal**:

```bash
ssh team6@192.168.2.113
docker exec -it $(docker ps -q) bash
source /ros2_ws/install/setup.bash
ros2 run nav2_map_server map_saver_cli -f /root/my_map
```

Copy the map out of Docker (so it's not lost when container stops):
```bash
# On the robot host (not inside Docker):
docker cp $(docker ps -q):/root/my_map.pgm ~/maps/my_map.pgm
docker cp $(docker ps -q):/root/my_map.yaml ~/maps/my_map.yaml
```

---

## 5. Useful Diagnostic Commands

Run these in a second terminal (`docker exec -it $(docker ps -q) bash`):

```bash
# Check what topics are running
ros2 topic list

# Check LIDAR is publishing
ros2 topic hz /scan

# Check odometry is publishing
ros2 topic hz /odom

# Check map is publishing (wait ~10s after launch)
ros2 topic hz /map

# Check full TF tree
ros2 run tf2_tools view_frames

# Check SLAM lifecycle state
ros2 lifecycle get /slam_toolbox
```

---

## 6. Rebuild and Deploy (after code changes on laptop)

```bash
# On your laptop — push changes to git:
cd /home/eddywin32/ros2_ws/src/p3at_robot
git push

# SSH into robot, pull and rebuild Docker:
ssh team6@192.168.2.113
cd ~/ros2_ws/src/p3at_robot
git pull
docker build -t pioneer_robot .
```

---

## 7. Stop Everything

Press `Ctrl+C` in the launch terminal. The `--rm` flag means the container deletes itself automatically.

To force-stop a stuck container:
```bash
docker stop $(docker ps -q)
```

---

## 8. Autonomous Waypoint Navigation (Nav2)

### How it works
- Press **X** on the controller to enter autonomous mode
- **Hold both triggers** at all times — releasing them immediately stops the robot (dead-man switch)
- Nav2 plans a path around obstacles and drives the robot smoothly to each goal

### Run waypoint navigation

Drive the robot to your desired starting position, then open a **second SSH terminal**:

```bash
ssh team6@192.168.2.113
```

If the launch is running inside Docker, attach to the container:
```bash
docker exec -it $(docker ps -q) bash
source /ros2_ws/install/setup.bash
```

If you are already inside the Docker container (prompt shows `root@`), skip the above and just run:
```bash
ros2 run p3at_robot nav2_waypoints.py
```

The script will:
1. Wait for Nav2 to be fully active
2. Read the robot's **current position and heading** from the map
3. Drive a 2×2 metre square relative to that position

Example output:
```
Nav2 is active!
Robot at (1.23, 0.45), heading 12.3 deg
Sending 4 waypoints...
Waypoint 1/4
Waypoint 2/4
...
All waypoints reached!
```

### Change the waypoints

Edit `src/nav2_waypoints.py` — waypoints are defined as `(forward, left, heading_offset_degrees)` relative to where the robot is when you run the script:

```python
relative_waypoints = [
    (2.0, 0.0,   0.0),   # 2m forward
    (2.0, 2.0,  90.0),   # 2m forward + 2m left, facing left
    (0.0, 2.0, 180.0),   # 2m left of start, facing back
    (0.0, 0.0,   0.0),   # back to start, original heading
]
```

After editing, rebuild and deploy (Section 6).
