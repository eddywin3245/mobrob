#!/usr/bin/env python3
"""
Orange cone detection and tracking.
Subscribes to the RGB camera feed and detects orange traffic cones.
Used for waypoint marker identification in autonomous navigation.

Topics:
  /camera/rgb/image_raw           — RGB input (subscribed)
  /camera/detected_cones          — Annotated image with bounding boxes (published)
  /camera/cone_detections         — Detection results (published)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import json


class ConeDetectionNode(Node):
    def __init__(self):
        super().__init__('cone_detection')

        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_header = None
        self.image_count = 0

        self.image_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.annotated_pub = self.create_publisher(Image, '/camera/detected_cones', 10)
        self.detection_pub = self.create_publisher(String, '/camera/cone_detections', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/camera/cone_markers', 10)

        # Check for orange cone every 0.2 seconds (5 Hz)
        self.timer = self.create_timer(0.2, self.detection_loop)

        self.get_logger().info('Cone detection node started')
        self.get_logger().info('  Subscribing to /camera/rgb/image_raw')
        self.get_logger().info('  Publishing annotated to /camera/detected_cones')
        self.get_logger().info('  Publishing markers to /camera/cone_markers')
        self.get_logger().info('  Distance estimated from bounding box size')

    def image_callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.latest_header = msg.header

    def detect_orange(self, frame):
        """Detect orange traffic cones in the frame.

        Returns:
            detected (bool): True if cones found
            on_right (bool): True if largest cone is on right side of frame
            boxes (list): [(x, y, w, h), ...] bounding rectangles of detected cones
        """
        # Split BGR channels for custom red detection
        b, g, r = cv2.split(frame)

        # Relaxed orange detection for better edge coverage
        mask = (r > 130) & (g < 80) & (b < 100) & ((r - g) > 70) & ((r - b) > 70)

        # Clean up: remove noise, fill gaps
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = mask.astype('uint8')
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        boxes = []

        if contours:
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 800:  # Larger minimum area
                    x, y, w, h = cv2.boundingRect(cnt)
                    # Cone shape: tall and narrow (height > 1.2x width) — relaxed for better detection
                    aspect_ratio = h / (w + 1)
                    if aspect_ratio > 1.2:  # Stricter cone shape requirement
                        boxes.append((x, y, w, h))

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
            return

        detected, on_right, boxes = self.detect_orange(self.latest_frame)

        annotated = self.latest_frame.copy()

        if detected:
            side = 'RIGHT' if on_right else 'LEFT'
            self.get_logger().info(f'Orange cone(s) detected on {side} — {len(boxes)} cone(s)')

            # Draw bounding boxes and estimate distances
            for (x, y, w, h) in boxes:
                cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 165, 255), 2)

                # Estimate distance from bounding box size (calibrated constant)
                distance_m = 3600 / (w * h + 1)
                distance_m = max(0.2, min(distance_m, 5.0))  # Clamp to 0.2-5m range

                label = f'Cone {distance_m:.2f}m'
                cv2.putText(annotated, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                           0.5, (0, 165, 255), 2)

            # Photo saving disabled for testing
            # filename = f'/tmp/cone_{self.image_count}.png'
            # cv2.imwrite(filename, annotated)
            # self.image_count += 1

            # Publish with distance estimates
            detection_list = []
            marker_array = MarkerArray()

            for idx, (x, y, w, h) in enumerate(boxes):
                distance_m = 15000 / (w * h + 1)
                distance_m = max(0.2, min(distance_m, 5.0))

                detection_list.append({
                    'x': int(x), 'y': int(y), 'w': int(w), 'h': int(h),
                    'distance_m': float(distance_m)
                })

                # Create marker for this cone
                marker = Marker()
                marker.header.frame_id = 'camera_rgb_optical_frame'
                marker.header.stamp = self.latest_header.stamp
                marker.id = idx
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD

                # Position at bounding box center
                center_x = x + w / 2
                center_y = y + h / 2
                marker.pose.position.x = distance_m  # Use distance as X (depth)
                marker.pose.position.y = -(center_x - self.latest_frame.shape[1] / 2) * distance_m / 320  # Camera X to world Y
                marker.pose.position.z = -(center_y - self.latest_frame.shape[0] / 2) * distance_m / 240   # Camera Y to world Z

                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.1
                marker.color.r = 1.0
                marker.color.g = 0.165
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.text = f'{distance_m:.2f}m'
                marker.lifetime.sec = 1

                marker_array.markers.append(marker)

            det_msg = String()
            det_msg.data = json.dumps({
                'detected': True,
                'count': len(boxes),
                'side': side,
                'cones': detection_list
            })
            self.detection_pub.publish(det_msg)
            self.marker_pub.publish(marker_array)
        else:
            det_msg = String()
            det_msg.data = json.dumps({'detected': False, 'count': 0})
            self.detection_pub.publish(det_msg)

            # Publish empty marker array to clear old markers
            marker_array = MarkerArray()
            self.marker_pub.publish(marker_array)

        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        annotated_msg.header = self.latest_header
        self.annotated_pub.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

