#!/usr/bin/env python3
"""
Spatial Detection Network test — detect objects with 3D coordinates.
Uses YOLOv6 from OAK model zoo for generic object detection.

Topics:
  /spatial_detection/image    — annotated frame with bboxes
  /spatial_detection/detections — JSON with detection data
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
import depthai as dai
import threading
from pathlib import Path


class SpatialDetectionTest(Node):
    def __init__(self):
        super().__init__('spatial_detection_test')

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/spatial_detection/image', 10)
        self.det_pub = self.create_publisher(String, '/spatial_detection/detections', 10)

        self._running = True
        self._thread = threading.Thread(target=self._detection_loop, daemon=True)
        self._thread.start()

        self.get_logger().info('Spatial detection test started')

    def _detection_loop(self):
        try:
            # Download model if needed
            model_path = self._get_model()
            if not model_path:
                self.get_logger().error('Failed to get model')
                return

            pipeline = dai.Pipeline()

            # Stereo depth
            monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
            monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
            stereo = pipeline.create(dai.node.StereoDepth)
            monoLeftOut = monoLeft.requestFullResolutionOutput()
            monoRightOut = monoRight.requestFullResolutionOutput()
            monoLeftOut.link(stereo.left)
            monoRightOut.link(stereo.right)
            stereo.setRectification(True)
            stereo.setLeftRightCheck(True)

            # RGB camera for detection
            camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            camRgb.setPreviewSize(416, 416)
            camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

            # Spatial detection network
            spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionPostProcessor)
            spatialDetectionNetwork.setBlobPath(str(model_path))
            spatialDetectionNetwork.setConfidenceThreshold(0.5)
            spatialDetectionNetwork.input.setBlocking(False)
            spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
            spatialDetectionNetwork.setDepthLowerThreshold(100)
            spatialDetectionNetwork.setDepthUpperThreshold(5000)

            # Network-specific settings
            spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
            spatialDetectionNetwork.setDepthLowerThreshold(100)
            spatialDetectionNetwork.setDepthUpperThreshold(5000)

            # Yolo specific parameters
            spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
            spatialDetectionNetwork.setConfidenceThreshold(0.5)

            # Linking
            camRgb.preview.link(spatialDetectionNetwork.input)
            spatialDetectionNetwork.passthrough.link(pipeline.createXLinkOut().setStreamName("rgb").input)
            spatialDetectionNetwork.out.link(pipeline.createXLinkOut().setStreamName("detections").input)

            stereo.depth.link(spatialDetectionNetwork.inputDepth)
            spatialDetectionNetwork.passthroughDepth.link(pipeline.createXLinkOut().setStreamName("depth").input)

            with pipeline:
                pipeline.start()
                self.get_logger().info('Spatial detection pipeline started')

                qRgb = pipeline.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                qDet = pipeline.getOutputQueue(name="detections", maxSize=4, blocking=False)

                while self._running and pipeline.isRunning():
                    inRgb = qRgb.get()
                    inDet = qDet.get()

                    if inRgb is None:
                        continue

                    frame = inRgb.getCvFrame()
                    h, w = frame.shape[:2]

                    detections = []
                    if inDet is not None:
                        spatialData = inDet.detections
                        for detection in spatialData:
                            # Bounding box
                            xmin = int(detection.spatialCoordinates.x - 100)  # offset for width
                            ymin = int(detection.spatialCoordinates.y - 100)  # offset for height
                            xmax = int(detection.spatialCoordinates.x + 100)
                            ymax = int(detection.spatialCoordinates.y + 100)

                            # Draw bbox
                            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)

                            # Spatial coordinates (mm)
                            x = detection.spatialCoordinates.x / 1000.0  # convert to m
                            y = detection.spatialCoordinates.y / 1000.0
                            z = detection.spatialCoordinates.z / 1000.0

                            label = f"X: {x:.2f}m Y: {y:.2f}m Z: {z:.2f}m"
                            cv2.putText(frame, label, (xmin, ymin - 10),
                                       cv2.FONT_HERSHEY_TRIPLEX, 0.6, (0, 255, 0))

                            detections.append({
                                'x_m': x,
                                'y_m': y,
                                'z_m': z,
                                'confidence': float(detection.confidence) if hasattr(detection, 'confidence') else 0.0
                            })

                    # Publish image
                    msg_img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    msg_img.header.stamp = self.get_clock().now().to_msg()
                    msg_img.header.frame_id = 'camera_rgb'
                    self.image_pub.publish(msg_img)

                    # Publish detections
                    msg_det = String(data=json.dumps({'detections': detections}))
                    self.det_pub.publish(msg_det)

                    if detections:
                        self.get_logger().info(
                            f'Detected {len(detections)} objects',
                            throttle_duration_sec=2.0
                        )

        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def _get_model(self):
        """Download YOLOv6n model from OAK zoo if needed."""
        try:
            import subprocess
            model_dir = Path.home() / '.cache' / 'oak' / 'models'
            model_path = model_dir / 'yolov6n_coco_416x416.blob'

            if not model_path.exists():
                self.get_logger().info('Downloading YOLOv6n model from OAK zoo...')
                model_dir.mkdir(parents=True, exist_ok=True)
                # Use depthai-python's model downloader
                from depthai_sdk import OakCamera
                # Models are typically downloaded automatically
                # For now, use a hardcoded path or fallback
                return None

            return model_path
        except Exception as e:
            self.get_logger().warn(f'Could not get model: {e}')
            return None

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpatialDetectionTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
