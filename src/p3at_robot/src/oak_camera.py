#!/usr/bin/env python3
import os
import threading
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import depthai as dai
from datetime import timedelta

# Approximate OAK-D S2 intrinsics at full mono resolution (1280×800)
_FX, _FY = 882.0, 882.0
_CX, _CY = 640.0, 400.0
_DEPTH_STEP = 4   # subsample factor for PointCloud2 (4 → ~80k pts max)
_MIN_DEPTH  = 0.3  # m
_MAX_DEPTH  = 3.5  # m


def _calib_path():
    src = '/ros2_ws/src/p3at_robot/config/imu_calibration.yaml'
    if os.path.isdir(os.path.dirname(src)):
        return src
    return os.path.join(
        get_package_share_directory('p3at_robot'), 'config', 'imu_calibration.yaml')


def _load_cal():
    try:
        with open(_calib_path()) as f:
            data = yaml.safe_load(f)
        return data.get('oak_d', {})
    except Exception:
        return {}


def _apply_cal(cal, accel, gyro, mag_T):
    gb = np.array(cal.get('gyro_bias',     [0.0, 0.0, 0.0]))
    ab = np.array(cal.get('accel_bias',    [0.0, 0.0, 0.0]))
    hi = np.array(cal.get('mag_hard_iron', [0.0, 0.0, 0.0]))
    si = np.array(cal.get('mag_soft_iron', [[1, 0, 0], [0, 1, 0], [0, 0, 1]]))
    return (
        np.asarray(accel, dtype=float) - ab,
        np.asarray(gyro,  dtype=float) - gb,
        si @ (np.asarray(mag_T, dtype=float) - hi),
    )


class OakCameraNode(Node):
    def __init__(self):
        super().__init__('oak_camera')
        self.bridge    = CvBridge()
        self.rgb_pub   = self.create_publisher(Image,       '/camera/rgb/image_raw', 10)
        self.depth_pub = self.create_publisher(PointCloud2, '/camera/depth/points',  10)
        self.imu_pub   = self.create_publisher(Imu,         '/camera/imu',           10)
        # No magnetometer — BMI270 is a 6-axis IMU (accel + gyro only)
        self._cal      = _load_cal()
        self._running  = True
        threading.Thread(target=self._pipeline, daemon=True).start()
        self.get_logger().info('OAK-D node starting (RGB + depth cloud + IMU)')

    # ------------------------------------------------------------------ pipeline

    def _pipeline(self):
        try:
            pipeline = dai.Pipeline()

            # RGB (320×240)
            cam_rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            rgb_q   = cam_rgb.requestOutput((320, 240), dai.ImgFrame.Type.BGR888p) \
                              .createOutputQueue()

            # Stereo depth
            mono_l = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
            mono_r = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
            stereo  = pipeline.create(dai.node.StereoDepth)
            mono_l.requestFullResolutionOutput().link(stereo.left)
            mono_r.requestFullResolutionOutput().link(stereo.right)
            stereo.setRectification(True)
            stereo.setLeftRightCheck(True)
            depth_q = stereo.depth.createOutputQueue()

            # IMU — BMI270 chip: only RAW outputs supported (no mag, no calibrated gyro)
            imu_node = pipeline.create(dai.node.IMU)
            imu_node.enableIMUSensor(
                [dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 100)
            imu_q = imu_node.out.createOutputQueue()

            with pipeline:
                pipeline.start()
                self.get_logger().info('OAK-D pipeline running (RGB + depth + IMU)')
                threads = [
                    threading.Thread(target=self._rgb_loop,   args=(rgb_q,),   daemon=True),
                    threading.Thread(target=self._depth_loop, args=(depth_q,), daemon=True),
                    threading.Thread(target=self._imu_loop,   args=(imu_q,),   daemon=True),
                ]
                for t in threads:
                    t.start()
                for t in threads:
                    t.join()

        except Exception as exc:
            import traceback
            self.get_logger().error(f'OAK-D pipeline error: {exc}\n{traceback.format_exc()}')

    # ------------------------------------------------------------------ RGB

    def _rgb_loop(self, q):
        while self._running:
            f = q.get(timedelta(milliseconds=200))
            if f is None:
                continue
            msg = self.bridge.cv2_to_imgmsg(f.getCvFrame(), encoding='bgr8')
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_rgb_optical_frame'
            self.rgb_pub.publish(msg)

    # ------------------------------------------------------------------ depth → PointCloud2

    def _depth_loop(self, q):
        while self._running:
            f = q.get(timedelta(milliseconds=200))
            if f is None:
                continue
            cloud = self._to_cloud(f.getFrame())
            if cloud is not None:
                self.depth_pub.publish(cloud)

    def _to_cloud(self, depth_mm):
        h, w   = depth_mm.shape
        ys, xs = np.mgrid[0:h:_DEPTH_STEP, 0:w:_DEPTH_STEP]
        d      = depth_mm[ys, xs].astype(np.float32) / 1000.0
        mask   = (d > _MIN_DEPTH) & (d < _MAX_DEPTH)
        if not np.any(mask):
            return None
        d, ys, xs = d[mask], ys[mask], xs[mask]
        # Optical frame: X right, Y down, Z forward
        pts = np.stack([
            (xs - _CX) * d / _FX,
            (ys - _CY) * d / _FY,
            d,
        ], axis=-1).astype(np.float32)

        msg             = PointCloud2()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_depth_optical_frame'
        msg.height      = 1
        msg.width       = len(pts)
        msg.fields      = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step   = 12
        msg.row_step     = 12 * len(pts)
        msg.is_dense     = True
        msg.data         = pts.tobytes()
        return msg

    # ------------------------------------------------------------------ IMU

    def _imu_loop(self, q):
        lat_a = lat_g = None
        while self._running:
            data = q.get(timedelta(milliseconds=200))
            if data is None:
                continue

            for pkt in data.packets:
                try:
                    a = pkt.acceleroMeter
                    lat_a = (a.x, a.y, a.z)
                except (AttributeError, RuntimeError):
                    pass
                try:
                    g = pkt.gyroscope
                    lat_g = (g.x, g.y, g.z)
                except (AttributeError, RuntimeError):
                    pass
            if lat_a is None or lat_g is None:
                continue

            a_c, g_c, _ = _apply_cal(self._cal, lat_a, lat_g, (0.0, 0.0, 0.0))
            stamp = self.get_clock().now().to_msg()

            imu_msg = Imu()
            imu_msg.header.stamp    = stamp
            imu_msg.header.frame_id = 'camera_imu_frame'
            imu_msg.linear_acceleration.x = float(a_c[0])
            imu_msg.linear_acceleration.y = float(a_c[1])
            imu_msg.linear_acceleration.z = float(a_c[2])
            imu_msg.angular_velocity.x    = float(g_c[0])
            imu_msg.angular_velocity.y    = float(g_c[1])
            imu_msg.angular_velocity.z    = float(g_c[2])
            imu_msg.linear_acceleration_covariance[0] = 0.006
            imu_msg.linear_acceleration_covariance[4] = 0.006
            imu_msg.linear_acceleration_covariance[8] = 0.006
            imu_msg.angular_velocity_covariance[0]    = 0.003
            imu_msg.angular_velocity_covariance[4]    = 0.003
            imu_msg.angular_velocity_covariance[8]    = 0.003
            imu_msg.orientation_covariance[0] = -1.0   # orientation not provided
            self.imu_pub.publish(imu_msg)

    # ------------------------------------------------------------------ lifecycle

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
