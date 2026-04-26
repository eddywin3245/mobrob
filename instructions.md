# ── START ROBOT ──────────────────────────────────────────────────────────────

docker run -it --rm --privileged --network host \
  -v /dev/input:/dev/input \
  -v /dev/bus/usb:/dev/bus/usb \
  -v ~/robot_data:/data \
  -v ~/ros2_ws/src/p3at_robot:/ros2_ws/src/p3at_robot \
  -e ROS_DOMAIN_ID=6 \
  pioneer_robot bash

colcon build --packages-select p3at_robot && source install/setup.bash
ros2 launch p3at_robot mapping.launch.py

# Open extra terminal in the same container:
docker exec -it $(docker ps -q) bash
source /ros2_ws/install/setup.bash


# ── IMU CALIBRATION ──────────────────────────────────────────────────────────
# Run ONCE before first use, or after moving the robot to a new environment.
# Needs 4 terminals (all inside the container via docker exec above).

# Terminal 1 — Phidget IMU
ros2 run p3at_robot phidget_imu.py

# Terminal 2 — OAK-D camera
ros2 run p3at_robot oak_camera.py

# Terminal 3 — Gamepad (drive during rotation phase)
ros2 run joy joy_node &
ros2 run p3at_robot gamepad_controller.py &
ros2 run p3at_robot ariaNode -rp /dev/ttyUSB0

# Terminal 4 — Calibration script (follow the prompts)
ros2 run p3at_robot imu_calibrate.py

# Phase 1 (15 s): robot flat and still — gyro + accel bias
# Phase 2 (60 s): drive in yaw circles + slight tilts — mag hard/soft iron
#   Slight tilts (10-15°) each side are enough for a heavy robot
# Phase 3: face robot north with phone compass → saves north offset
#
# Restart sensor nodes after calibration to apply new values.