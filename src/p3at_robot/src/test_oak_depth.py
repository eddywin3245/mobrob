#!/usr/bin/env python3
"""
OAK-D depth test — using working depthai 3.5 API from OAK team.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import depthai as dai


class OakDepthTest(Node):
    def __init__(self):
        super().__init__('oak_depth_test')
        self.bridge = CvBridge()
        self.raw_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.vis_pub = self.create_publisher(Image, '/camera/depth/image_vis', 10)
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        self.get_logger().info('OAK depth test started')

    def _capture_loop(self):
        try:
            pipeline = dai.Pipeline()

            # Stereo cameras (left + right for depth)
            monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
            monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
            stereo = pipeline.create(dai.node.StereoDepth)

            # Linking
            monoLeftOut = monoLeft.requestFullResolutionOutput()
            monoRightOut = monoRight.requestFullResolutionOutput()
            monoLeftOut.link(stereo.left)
            monoRightOut.link(stereo.right)

            stereo.setRectification(True)
            stereo.setExtendedDisparity(True)
            stereo.setLeftRightCheck(True)

            disparityQueue = stereo.disparity.createOutputQueue()

            colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
            colorMap[0] = [0, 0, 0]

            with pipeline:
                pipeline.start()
                self.get_logger().info('Stereo depth pipeline started')
                maxDisparity = 1

                while self._running and pipeline.isRunning():
                    disparity = disparityQueue.get()
                    if disparity is None:
                        continue

                    npDisparity = disparity.getFrame()
                    maxDisparity = max(maxDisparity, np.max(npDisparity))

                    # Convert disparity to depth (mm)
                    # Approximate: depth_mm = baseline_mm * focal_length / disparity
                    # For OAK-D: ~375mm * 400 pixels / disparity
                    depth_mm = (150 * 400 / np.clip(npDisparity, 1, 255)).astype(np.uint16)
                    depth_clipped = np.clip(depth_mm, 0, 5000).astype(np.uint16)

                    # Publish raw depth
                    msg_raw = self.bridge.cv2_to_imgmsg(depth_clipped, encoding='mono16')
                    msg_raw.header.stamp = self.get_clock().now().to_msg()
                    msg_raw.header.frame_id = 'camera_depth'
                    self.raw_pub.publish(msg_raw)

                    # Colorized disparity
                    colorizedDisparity = cv2.applyColorMap(
                        ((npDisparity / maxDisparity) * 255).astype(np.uint8), colorMap)

                    msg_vis = self.bridge.cv2_to_imgmsg(colorizedDisparity, encoding='bgr8')
                    msg_vis.header.stamp = self.get_clock().now().to_msg()
                    msg_vis.header.frame_id = 'camera_depth'
                    self.vis_pub.publish(msg_vis)

                    valid = depth_clipped[depth_clipped > 0]
                    if len(valid) > 0:
                        self.get_logger().info(
                            f'Depth: min={np.min(valid)} mm, max={np.max(valid)} mm',
                            throttle_duration_sec=2.0
                        )

        except Exception as e:
            self.get_logger().error(f'Depth error: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OakDepthTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
