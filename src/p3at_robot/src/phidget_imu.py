#!/usr/bin/env python3
import math
import os
import threading
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from ament_index_python.packages import get_package_share_directory

try:
    from Phidget22.Devices.Spatial import Spatial
    from Phidget22.PhidgetException import PhidgetException
    PHIDGET_AVAILABLE = True
except ImportError:
    PHIDGET_AVAILABLE = False


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
        return data.get('phidget', {})
    except Exception:
        return {}


class PhidgetImuNode(Node):
    def __init__(self):
        super().__init__('phidget_imu')
        self.imu_pub = self.create_publisher(Imu,           '/imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag',       10)

        cal = _load_cal()
        self._gyro_bias  = np.array(cal.get('gyro_bias',     [0.0, 0.0, 0.0]))
        self._accel_bias = np.array(cal.get('accel_bias',    [0.0, 0.0, 0.0]))
        self._mag_hi     = np.array(cal.get('mag_hard_iron', [0.0, 0.0, 0.0]))
        self._mag_si     = np.array(cal.get('mag_soft_iron', [[1,0,0],[0,1,0],[0,0,1]]))

        self._lock   = threading.Lock()
        self._latest = None  # (acceleration, angularRate, magneticField)

        if not PHIDGET_AVAILABLE:
            self.get_logger().error('Phidget22 not installed — pip install Phidget22')
            return

        self.spatial = Spatial()
        self.spatial.setOnSpatialDataHandler(self._on_data)

        try:
            self.spatial.openWaitForAttachment(5000)
            self.spatial.setDataInterval(10)  # 100 Hz
            self.get_logger().info('Phidget Spatial 3/3/3 connected at 100 Hz')
        except PhidgetException as e:
            self.get_logger().error(f'Phidget connection failed: {e.details}')
            return

        self.create_timer(0.01, self._publish)  # 100 Hz

    def _on_data(self, spatial, acceleration, angularRate, magneticField, timestamp):
        with self._lock:
            self._latest = (acceleration, angularRate, magneticField)

    def _publish(self):
        with self._lock:
            data = self._latest
        if data is None:
            return

        acceleration, angularRate, magneticField = data
        stamp = self.get_clock().now().to_msg()

        # Unit conversion: g → m/s², deg/s → rad/s, Gauss → T
        accel_raw = np.array(acceleration) * 9.81
        gyro_raw  = np.array([math.radians(v) for v in angularRate])
        mag_raw   = np.array(magneticField) * 1e-4   # G → T

        # Apply calibration
        accel = accel_raw - self._accel_bias
        gyro  = gyro_raw  - self._gyro_bias
        mag   = self._mag_si @ (mag_raw - self._mag_hi)

        imu_msg = Imu()
        imu_msg.header.stamp    = stamp
        imu_msg.header.frame_id = 'base_link'

        imu_msg.linear_acceleration.x = float(accel[0])
        imu_msg.linear_acceleration.y = float(accel[1])
        imu_msg.linear_acceleration.z = float(accel[2])
        imu_msg.linear_acceleration_covariance[0] = 0.004
        imu_msg.linear_acceleration_covariance[4] = 0.004
        imu_msg.linear_acceleration_covariance[8] = 0.004

        imu_msg.angular_velocity.x = float(gyro[0])
        imu_msg.angular_velocity.y = float(gyro[1])
        imu_msg.angular_velocity.z = float(gyro[2])
        imu_msg.angular_velocity_covariance[0] = 0.002
        imu_msg.angular_velocity_covariance[4] = 0.002
        imu_msg.angular_velocity_covariance[8] = 0.002

        imu_msg.orientation_covariance[0] = -1.0   # EKF computes orientation
        self.imu_pub.publish(imu_msg)

        mag_msg = MagneticField()
        mag_msg.header.stamp    = stamp
        mag_msg.header.frame_id = 'base_link'
        mag_msg.magnetic_field.x = float(mag[0])
        mag_msg.magnetic_field.y = float(mag[1])
        mag_msg.magnetic_field.z = float(mag[2])
        mag_msg.magnetic_field_covariance[0] = 1e-6
        mag_msg.magnetic_field_covariance[4] = 1e-6
        mag_msg.magnetic_field_covariance[8] = 1e-6
        self.mag_pub.publish(mag_msg)

    def destroy_node(self):
        if PHIDGET_AVAILABLE and hasattr(self, 'spatial'):
            try:
                self.spatial.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PhidgetImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
