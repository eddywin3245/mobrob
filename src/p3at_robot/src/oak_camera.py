#!/usr/bin/env python3
"""OAK-D RGB publisher. (Device is OAK-D-S2 — no IMU onboard.)"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import threading
from datetime import timedelta


class OakCameraNode(Node):
    def __init__(self):
        super().__init__('oak_camera')

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self._running = True

        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        self.get_logger().info('OAK camera node started — publishing to /camera/rgb/image_raw')

    def _capture_loop(self):
        try:
            with dai.Device() as device:
                pipeline = dai.Pipeline(device)

                cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
                preview = cam.requestOutput((320, 240), dai.ImgFrame.Type.BGR888p)
                queue = preview.createOutputQueue()

                pipeline.start()

                while self._running and pipeline.isRunning():
                    img_frame = queue.get(timedelta(milliseconds=100))
                    if img_frame is None:
                        continue
                    img = img_frame.getCvFrame()
                    msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'camera_rgb_frame'
                    self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f'OAK camera error: {e}')

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OakCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
