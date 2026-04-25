#!/usr/bin/env python3
import math
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

try:
    from Phidget22.Devices.Spatial import Spatial
    from Phidget22.PhidgetException import PhidgetException
    PHIDGET_AVAILABLE = True
except ImportError:
    PHIDGET_AVAILABLE = False


class PhidgetImuNode(Node):
    def __init__(self):
        super().__init__('phidget_imu')
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)

        self._lock = threading.Lock()
        self._latest = None  # (acceleration, angularRate, magneticField)

        if not PHIDGET_AVAILABLE:
            self.get_logger().error(
                'Phidget22 not installed — run: pip install Phidget22')
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
        # Called from Phidget thread — just store, publish from ROS timer
        with self._lock:
            self._latest = (acceleration, angularRate, magneticField)

    def _publish(self):
        with self._lock:
            data = self._latest

        if data is None:
            return

        acceleration, angularRate, magneticField = data
        stamp = self.get_clock().now().to_msg()

        # --- IMU message ---
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = 'base_link'

        # Linear acceleration: g -> m/s²
        imu_msg.linear_acceleration.x = acceleration[0] * 9.81
        imu_msg.linear_acceleration.y = acceleration[1] * 9.81
        imu_msg.linear_acceleration.z = acceleration[2] * 9.81
        imu_msg.linear_acceleration_covariance[0] = 0.004
        imu_msg.linear_acceleration_covariance[4] = 0.004
        imu_msg.linear_acceleration_covariance[8] = 0.004

        # Angular velocity: deg/s -> rad/s
        imu_msg.angular_velocity.x = math.radians(angularRate[0])
        imu_msg.angular_velocity.y = math.radians(angularRate[1])
        imu_msg.angular_velocity.z = math.radians(angularRate[2])
        imu_msg.angular_velocity_covariance[0] = 0.002
        imu_msg.angular_velocity_covariance[4] = 0.002
        imu_msg.angular_velocity_covariance[8] = 0.002

        # Orientation not computed here — EKF handles that
        imu_msg.orientation_covariance[0] = -1.0

        self.imu_pub.publish(imu_msg)

        # --- Magnetometer message ---
        # Phidget returns Gauss; ROS uses Tesla (1 G = 1e-4 T)
        mag_msg = MagneticField()
        mag_msg.header.stamp = stamp
        mag_msg.header.frame_id = 'base_link'
        mag_msg.magnetic_field.x = magneticField[0] * 1e-4
        mag_msg.magnetic_field.y = magneticField[1] * 1e-4
        mag_msg.magnetic_field.z = magneticField[2] * 1e-4
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
