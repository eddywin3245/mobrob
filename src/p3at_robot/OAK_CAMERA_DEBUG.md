# OAK-D Camera Debugging Guide

This guide helps troubleshoot OAK-D camera issues in the robot system.

## Recent Improvements

The oak_camera.py node now includes:
- **Device enumeration**: Lists all connected OAK devices before attempting connection
- **Explicit device selection**: Uses DeviceInfo to be more specific about which device to connect
- **Exponential backoff**: Waits longer between retries specifically for "already in use" errors
- **Better diagnostics**: Logs device MXIDs and provides actionable error messages

## Build and Deploy

```bash
# On your laptop/development machine
git pull origin part2-robot
git push  # Ensure changes are on GitHub

# On the robot
cd ~/ros2_ws
git pull origin part2-robot

# Rebuild container
docker build -t p3at_robot:latest --no-cache -f src/p3at_robot/Dockerfile src/p3at_robot

# Run container with privileged access for USB/hardware
docker run -it --privileged --net=host -v /dev:/dev p3at_robot:latest
```

Inside container:
```bash
# Source setup
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

# Launch full system
ros2 launch p3at_robot gamepad.launch.py

# Or test just the camera in another terminal
ros2 run p3at_robot oak_camera.py
```

## Expected Log Output (Success)

```
[INFO] Connecting to OAK device (attempt 1/10)
[INFO] Found 1 device(s): ['4.1.4']
[INFO] Attempting to connect to device: 4.1.4
[INFO] OAK device connected successfully!
[INFO] Publishing RGB to /camera/rgb/image_raw
[INFO] Publishing depth to /camera/depth/image_raw
```

Then RGB and depth frames should publish:
```bash
# In another terminal
ros2 topic hz /camera/rgb/image_raw  # Should show ~30 Hz
ros2 topic hz /camera/depth/image_raw  # Should show ~30 Hz
```

## Common Issues

### Issue: "X_LINK_DEVICE_ALREADY_IN_USE" Error

```
[WARN] OAK device error (attempt 1/10): Cannot connect to device with name "4.1.4", 
       it is used by another process. Error: X_LINK_DEVICE_ALREADY_IN_USE
```

**Causes:**
- Device wasn't properly released from previous connection
- Another depthai process still holding the device
- USB device state corrupted

**Solutions (in order):**

1. **Wait for retries** (automatic)
   - With the new exponential backoff, it waits 2s, 4s, 6s, etc. between retries
   - Give it up to ~1 minute for all 10 retries

2. **Restart the container**
   ```bash
   # Exit current container, then restart
   docker run -it --privileged --net=host -v /dev:/dev p3at_robot:latest
   ```

3. **Unplug and replug the OAK camera**
   - Physical USB disconnect/reconnect resets device state
   - Wait 5 seconds after unplugging before replugging

4. **Check for zombie processes** (inside container)
   ```bash
   ps aux | grep oak_camera
   ps aux | grep depthai
   # If you see old processes, kill them:
   pkill -f oak_camera
   pkill -f depthai
   ```

### Issue: No Devices Found

```
[ERROR] OAK device not found. Check USB connection.
```

**Causes:**
- Camera not plugged in
- USB cable issue
- Docker container doesn't have device access

**Solutions:**

1. Check USB connection
   ```bash
   # Inside container
   lsusb | grep "Luxvisions\|Movidius"
   # Should show OAK-D device
   ```

2. Ensure container runs with `--privileged` and `-v /dev:/dev`

3. Try the device diagnostic script:
   ```bash
   # Inside container
   python3 /ros2_ws/src/p3at_robot/src/test_oak_device.py
   ```

### Issue: Depth Pipeline Failed

```
[WARN] Depth pipeline failed, using RGB only: ...
```

This is non-critical. Camera will operate with:
- ✅ RGB frames publishing to `/camera/rgb/image_raw`
- ❌ Depth frames NOT publishing

If you need depth:
- Check camera hardware (left/right cameras may be damaged)
- Try a different OAK-D model/firmware version

## Testing Individual Components

### Test 1: Device Detection
```bash
python3 /ros2_ws/src/p3at_robot/src/test_oak_device.py
```

### Test 2: RGB and Depth Topics
```bash
ros2 topic list | grep camera
# Should see:
# /camera/rgb/image_raw
# /camera/rgb/camera_info
# /camera/depth/image_raw (if depth working)
# /camera/depth/camera_info (if depth working)

# Check data is publishing
ros2 topic hz /camera/rgb/image_raw
```

### Test 3: Cone Detection
```bash
ros2 topic list | grep cone
# Should see:
# /camera/detected_cones
# /camera/cone_detections

# Watch for cone detections
ros2 topic echo /camera/cone_detections
# Should see JSON messages with detection info
```

### Test 4: Foxglove Visualization
1. Connect to Foxglove (port 8765)
2. Subscribe to `/camera/detected_cones` (should show annotated RGB with bounding boxes)
3. Subscribe to `/camera/depth/image_raw` (if depth working)
4. Check cone detection results in `/camera/cone_detections`

## If All Else Fails

1. **Full system restart**
   ```bash
   docker rm -f $(docker ps -aq)  # Remove all containers
   docker build -t p3at_robot:latest --no-cache -f src/p3at_robot/Dockerfile src/p3at_robot
   # Start fresh
   ```

2. **Camera replacement**
   - If device has firmware issues or hardware problems
   - Physical unplug/replug is a good first test

3. **Hardware diagnostics**
   - Try camera on a different USB port
   - Test on a different computer if possible
   - Check OAK-D firmware version

## Notes

- Camera frames are published at ~30 FPS (depends on device)
- RGB intrinsics default: fx=430, fy=430, cx=320, cy=240 (640×480)
- Depth intrinsics default: fx=388, fy=388, cx=320, cy=240 (640×480)
- Frame IDs: `camera_rgb_optical_frame`, `camera_depth_optical_frame`
- TF transforms position camera at x=0.15m, z=0.1m from base_link
