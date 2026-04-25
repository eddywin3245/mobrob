#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

KNOWN_DISTANCE = 200.0  # cm
KNOWN_HEIGHT = 30.5      # cm
pixel_height_in_calibration = 150

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.bridge = CvBridge()
        self.latest_frame = None
        self.image_count = 0

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Check for orange cone every 0.5 seconds
        self.timer = self.create_timer(0.5, self.detection_loop)

        self.get_logger().info('Camera node started — subscribing to /camera/image_raw')

    def image_callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def detect_orange(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_orange = (1, 100, 100)
        upper_orange = (20, 255, 255)

        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        boxes = []

        if contours:
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 500:
                    boxes.append(cv2.boundingRect(cnt))

            if boxes:
                # Use the largest box to determine left/right side
                largest_box = max(boxes, key=lambda b: b[2] * b[3])
                x, y, w, h = largest_box
                center_x = x + w // 2
                width = frame.shape[1]

                return True, center_x > width / 2, boxes

        return False, False, []

    def detection_loop(self):
        if self.latest_frame is None:
            self.get_logger().warn('No camera frame received yet...')
            return

        detected, on_right, boxes = self.detect_orange(self.latest_frame)

        if detected:
            side = 'RIGHT' if on_right else 'LEFT'
            self.get_logger().info(f'Orange cone detected on {side}')

            annotated = self.latest_frame.copy()
            for (x, y, w, h) in boxes:
                cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 165, 255), 2)

            filename = f'/tmp/cone_{self.image_count}.png'
            cv2.imwrite(filename, annotated)
            self.get_logger().info(f'Image saved: {filename}')
            self.image_count += 1
        else:
            self.get_logger().info('No orange cone detected')


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()