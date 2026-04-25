#!/usr/bin/env python3
"""
RGBD Data Capture for Neural Network Training

Captures RGB and depth frames from OAK-D camera and saves them to disk.
Organizes data into rgb/ and depth/ directories with synchronized naming.

Usage:
  python3 capture_rgbd.py --output ./rgbd_data --count 500

Keys:
  SPACE - capture frame
  R     - record continuously (press again to stop)
  Q     - quit
"""
import cv2
import depthai as dai
import numpy as np
from pathlib import Path
import argparse
import json
from datetime import datetime


def capture_rgbd(output_dir, max_frames=None):
    """Capture RGBD data from OAK-D camera."""

    output_dir = Path(output_dir)
    rgb_dir = output_dir / 'rgb'
    depth_dir = output_dir / 'depth'
    rgb_dir.mkdir(parents=True, exist_ok=True)
    depth_dir.mkdir(parents=True, exist_ok=True)

    print(f"Saving to: {output_dir}")
    print(f"  RGB:   {rgb_dir}")
    print(f"  Depth: {depth_dir}")

    # Create pipeline
    pipeline = dai.Pipeline()

    # RGB camera
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    cam_out = cam.requestOutput((640, 400)).createOutputQueue()

    # Stereo depth
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo = pipeline.create(dai.node.StereoDepth)

    monoLeftOut = monoLeft.requestFullResolutionOutput()
    monoRightOut = monoRight.requestFullResolutionOutput()
    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)

    stereo.setRectification(True)
    stereo.setExtendedDisparity(True)
    stereo.setLeftRightCheck(True)

    depth_queue = stereo.disparity.createOutputQueue()

    frame_count = 0
    recording = False
    colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
    colorMap[0] = [0, 0, 0]
    maxDisparity = 1

    # Calibration data
    calib_data = {}

    with pipeline:
        pipeline.start()
        print("\n=== RGBD Capture Started ===")
        print("SPACE - Capture single frame")
        print("R     - Toggle continuous recording")
        print("Q     - Quit\n")

        while True:
            rgb_frame = cam_out.get()
            depth_frame = depth_queue.get()

            if rgb_frame is None or depth_frame is None:
                continue

            rgb = rgb_frame.getCvFrame()
            npDisparity = depth_frame.getFrame()
            maxDisparity = max(maxDisparity, np.max(npDisparity))

            # Colorize depth for preview
            colorizedDepth = cv2.applyColorMap(
                ((npDisparity / maxDisparity) * 255).astype(np.uint8),
                colorMap)

            # Convert disparity to depth (mm)
            depth_mm = (150 * 400 / np.clip(npDisparity, 1, 255)).astype(np.uint16)
            depth_clipped = np.clip(depth_mm, 0, 5000).astype(np.uint16)

            # Display
            display = np.hstack([rgb, colorizedDepth])
            status = f"Captured: {frame_count} | Recording: {'ON' if recording else 'OFF'}"
            cv2.putText(display, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                       0.7, (0, 255, 0), 2)
            cv2.imshow('RGBD Capture - SPACE/R/Q', display)

            key = cv2.waitKey(1) & 0xFF

            should_capture = False
            if key == ord(' '):  # Space - single capture
                should_capture = True
            elif key == ord('r'):  # R - toggle recording
                recording = not recording
                print(f"Recording: {'ON' if recording else 'OFF'}")
            elif key == ord('q'):  # Q - quit
                break

            if recording or should_capture:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                frame_id = f"{frame_count:06d}_{timestamp}"

                # Save RGB
                rgb_path = rgb_dir / f"{frame_id}.png"
                cv2.imwrite(str(rgb_path), rgb)

                # Save depth (as uint16 PNG - preserves mm values)
                depth_path = depth_dir / f"{frame_id}.png"
                cv2.imwrite(str(depth_path), depth_clipped)

                frame_count += 1
                print(f"Captured {frame_count}: {frame_id}")

                if max_frames and frame_count >= max_frames:
                    print(f"Reached max frames ({max_frames})")
                    break

    # Save metadata
    metadata = {
        'total_frames': frame_count,
        'rgb_format': 'BGR8',
        'depth_format': 'uint16_mm',
        'camera_notes': 'OAK-D RGB (640x400) + Stereo Depth'
    }

    with open(output_dir / 'metadata.json', 'w') as f:
        json.dump(metadata, f, indent=2)

    print(f"\n=== Capture Complete ===")
    print(f"Total frames: {frame_count}")
    print(f"Output: {output_dir}")
    print(f"Metadata: {output_dir / 'metadata.json'}")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Capture RGBD data from OAK-D')
    parser.add_argument('--output', type=str, default='./rgbd_data',
                       help='Output directory for RGBD data')
    parser.add_argument('--count', type=int, default=None,
                       help='Max frames to capture (None = unlimited)')

    args = parser.parse_args()

    try:
        capture_rgbd(args.output, args.count)
    except KeyboardInterrupt:
        print("\nInterrupted")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
