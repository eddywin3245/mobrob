#!/usr/bin/env python3
"""
Dual-sensor IMU calibration: Phidget Spatial 3/3/3 + OAK-D BMI270.
See instructions.md for how to run.
"""

import os
import sys
import time
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from ament_index_python.packages import get_package_share_directory

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAVE_MPL = True
except ImportError:
    HAVE_MPL = False

STATIC_SECS   = 15
ROTATION_SECS = 60


# ------------------------------------------------------------------ paths

def _src_config_dir():
    return '/ros2_ws/src/p3at_robot/config'


def _share_config_dir():
    return os.path.join(get_package_share_directory('p3at_robot'), 'config')


def _primary_save_path():
    d = _src_config_dir()
    if os.path.isdir(d):
        return os.path.join(d, 'imu_calibration.yaml')
    return os.path.join(_share_config_dir(), 'imu_calibration.yaml')


# ------------------------------------------------------------------ ellipsoid math

def _fit_ellipsoid(samples):
    """
    Algebraic least-squares ellipsoid fit.
    Returns (hard_iron [3], soft_iron W [3×3]).
    Apply: corrected = W @ (raw - hard_iron)  →  unit sphere
    """
    if len(samples) < 20:
        return np.zeros(3), np.eye(3)

    x, y, z = samples[:, 0], samples[:, 1], samples[:, 2]
    D = np.column_stack([
        x*x, y*y, z*z,
        2*x*y, 2*x*z, 2*y*z,
        2*x,   2*y,   2*z,
    ])
    theta, _, _, _ = np.linalg.lstsq(D, np.ones(len(samples)), rcond=None)

    A, B, C   = theta[0], theta[1], theta[2]
    Dv, E, F  = theta[3], theta[4], theta[5]
    G, H, I   = theta[6], theta[7], theta[8]

    M = np.array([
        [A,  Dv, E,  G],
        [Dv, B,  F,  H],
        [E,  F,  C,  I],
        [G,  H,  I, -1.0],
    ])
    M33     = M[:3, :3]
    center  = -np.linalg.solve(M33, M[:3, 3])      # hard iron (ellipsoid centre)

    # Shape matrix: (m-h)^T A_shape (m-h) = 1
    denom   = center @ M33 @ center - M[3, 3]
    A_shape = (M33 / denom)
    A_shape = 0.5 * (A_shape + A_shape.T)           # ensure symmetric

    eigenvalues, V = np.linalg.eigh(A_shape)
    eigenvalues = np.maximum(eigenvalues, 1e-12)
    W = V @ np.diag(np.sqrt(eigenvalues)) @ V.T     # soft-iron matrix

    return center, W


def _sphere_rms(samples, hard_iron, soft_iron):
    """RMS deviation from a unit sphere after correction (lower is better)."""
    corrected = (samples - hard_iron) @ soft_iron.T
    radii = np.linalg.norm(corrected, axis=1)
    return float(np.sqrt(np.mean((radii - radii.mean()) ** 2)))


# ------------------------------------------------------------------ node

class ImuCalibrator(Node):
    def __init__(self):
        super().__init__('imu_calibrate')
        self._phase = 'idle'
        self._lock  = threading.Lock()

        self.ph_gyro     = []
        self.ph_accel    = []
        self.ph_mag      = []
        self.ph_mag_north = []   # samples collected while facing north
        self.oak_gyro  = []
        self.oak_accel = []

        self.create_subscription(Imu,          '/imu/data_raw', self._ph_imu_cb,  10)
        self.create_subscription(MagneticField, '/imu/mag',      self._ph_mag_cb,  10)
        self.create_subscription(Imu,          '/camera/imu',   self._oak_imu_cb, 10)
        # OAK-D has BMI270 (6-axis only) — no magnetometer topic

        self._done = threading.Event()

    # ---------------------------------------------------------- callbacks

    def _ph_imu_cb(self, msg):
        with self._lock:
            if self._phase == 'static':
                self.ph_gyro.append([
                    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
                self.ph_accel.append([
                    msg.linear_acceleration.x, msg.linear_acceleration.y,
                    msg.linear_acceleration.z])

    def _ph_mag_cb(self, msg):
        with self._lock:
            if self._phase in ('static', 'rotation'):
                self.ph_mag.append([
                    msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])
            elif self._phase == 'north':
                self.ph_mag_north.append([
                    msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])

    def _oak_imu_cb(self, msg):
        with self._lock:
            if self._phase == 'static':
                self.oak_gyro.append([
                    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
                self.oak_accel.append([
                    msg.linear_acceleration.x, msg.linear_acceleration.y,
                    msg.linear_acceleration.z])

    # No _oak_mag_cb — BMI270 has no magnetometer

    # ---------------------------------------------------------- calibration flow

    def run(self):
        print('\n' + '=' * 62)
        print('   DUAL-SENSOR IMU CALIBRATION')
        print('   Phidget Spatial 3/3/3  +  OAK-D BNO085')
        print('=' * 62)
        print('\nPrerequisites:')
        print('  • phidget_imu.py running  → /imu/data_raw, /imu/mag')
        print('  • oak_camera.py running   → /camera/imu  (BMI270: no mag)')
        print('  • imu_calibration.yaml has default (identity) values\n')

        # ---- wait for data to arrive ----
        print('Waiting for sensor data (up to 10 s)...')
        deadline = time.time() + 10.0
        while time.time() < deadline:
            with self._lock:
                got_ph  = len(self.ph_gyro)  > 0
                got_oak = len(self.oak_gyro) > 0
            if got_ph and got_oak:
                break
            time.sleep(0.2)
        with self._lock:
            got_ph  = len(self.ph_gyro)  > 0
            got_oak = len(self.oak_gyro) > 0
        if not got_ph:
            print('  [WARN] No Phidget data — will calibrate OAK-D only')
        if not got_oak:
            print('  [WARN] No OAK-D IMU data — will calibrate Phidget only')

        # Clear pre-wait junk
        with self._lock:
            for lst in (self.ph_gyro, self.ph_accel, self.ph_mag,
                        self.oak_gyro, self.oak_accel):
                lst.clear()

        # ---- PHASE 1: static ----
        print('\n[1/2] STATIC CALIBRATION  (gyro + accel bias)')
        print('  Set the robot on a flat, level surface.')
        print('  The robot must be completely motionless.')
        input('  → Press ENTER when ready...')
        print(f'  Collecting {STATIC_SECS} s of static data', end='', flush=True)
        with self._lock:
            self._phase = 'static'
        for _ in range(STATIC_SECS):
            time.sleep(1)
            print('.', end='', flush=True)
        with self._lock:
            self._phase = 'idle'
        print('  done.')

        # ---- PHASE 2: rotation ----
        print('\n[2/2] MAGNETOMETER CALIBRATION  (hard-iron + soft-iron)')
        print('  Drive/rotate the robot to cover as much of the sphere as possible.')
        print('  You can use the gamepad controller (run it in another terminal).')
        print('  1. At least 2 full circles in place (yaw) — both directions.')
        print('  2. Slight tilts in each direction are enough — 10-15° front/back/left/right.')
        print('     Even small tilts improve the 3-D ellipsoid fit significantly.')
        print('  Move continuously; avoid stopping for more than 1 second.')
        input(f'  → Press ENTER to start ({ROTATION_SECS} s)...')
        print(f'  Collecting {ROTATION_SECS} s of rotation data', end='', flush=True)
        with self._lock:
            self._phase = 'rotation'
        for _ in range(ROTATION_SECS):
            time.sleep(1)
            print('.', end='', flush=True)
        with self._lock:
            self._phase = 'idle'
        print('  done.')

        # ---- compute ----
        result = self._compute()

        # ---- PHASE 3: north reference ----
        print('\n[3/3] NORTH REFERENCE  (magnetic heading offset)')
        print('  Face the robot towards magnetic north.')
        print('  Use a compass app on your phone as reference.')
        print('  The robot must be stationary.')
        resp = input('  → Press ENTER to sample (3 s), or type "skip" to save 0.0: ')
        if resp.strip().lower() == 'skip':
            result['phidget']['mag_north_offset'] = 0.0
            print('  Skipped — north offset saved as 0.0 rad')
        else:
            print('  Sampling 3 s of magnetometer data...', end='', flush=True)
            with self._lock:
                self._phase = 'north'
            for _ in range(3):
                time.sleep(1)
                print('.', end='', flush=True)
            with self._lock:
                self._phase = 'idle'
            north_offset = self._compute_north_offset(result)
            result['phidget']['mag_north_offset'] = north_offset
            print(f'  done.  North offset: {np.degrees(north_offset):.1f}°')

        # ---- save ----
        self._save(result)
        if HAVE_MPL:
            self._plot(result)

        self._done.set()

    # ---------------------------------------------------------- compute

    def _compute(self):
        result = {}

        # oak_d has no magnetometer (BMI270 = 6-axis only), pass empty list for mag
        for key, label, gyro_l, accel_l, mag_l in [
            ('phidget', 'Phidget', self.ph_gyro,  self.ph_accel,  self.ph_mag),
            ('oak_d',   'OAK-D',   self.oak_gyro, self.oak_accel, []),
        ]:
            print(f'--- {label} ---')

            if len(gyro_l) > 20:
                g_arr = np.array(gyro_l)
                a_arr = np.array(accel_l)
                gyro_bias  = g_arr.mean(axis=0).tolist()
                mean_a     = a_arr.mean(axis=0)
                norm_a     = np.linalg.norm(mean_a)
                # Preserve direction, correct magnitude to standard g
                accel_bias = (mean_a - mean_a / norm_a * 9.80665).tolist()
                print(f'  Samples: {len(g_arr)}')
                print(f'  Gyro bias  (rad/s): [{", ".join(f"{v:+.5f}" for v in gyro_bias)}]')
                print(f'  Accel bias (m/s²):  [{", ".join(f"{v:+.5f}" for v in accel_bias)}]')
                print(f'  Accel magnitude (m/s²): {norm_a:.4f}  (expect 9.807)')
            else:
                gyro_bias  = [0.0, 0.0, 0.0]
                accel_bias = [0.0, 0.0, 0.0]
                print(f'  Static data insufficient ({len(gyro_l)} samples) — zero biases used')

            if len(mag_l) > 100:
                m_arr = np.array(mag_l)
                # Check Z coverage for 3-D quality warning
                z_std = float(np.std(m_arr[:, 2]))
                if z_std < 2e-6:
                    print('  [WARN] Low Z-axis coverage — tilt robot for better 3-D calibration')
                hard_iron, soft_iron = _fit_ellipsoid(m_arr)
                rms = _sphere_rms(m_arr, hard_iron, soft_iron)
                print(f'  Mag samples: {len(m_arr)}')
                print(f'  Hard-iron (T): [{", ".join(f"{v:.3e}" for v in hard_iron)}]')
                print(f'  Sphere RMS:    {rms:.2e} T  (aim < 1e-6 T)')
            else:
                hard_iron = np.zeros(3)
                soft_iron = np.eye(3)
                print(f'  Mag data insufficient ({len(mag_l)} samples) — identity used')

            entry = {
                'gyro_bias':     [round(v, 8) for v in gyro_bias],
                'accel_bias':    [round(v, 8) for v in accel_bias],
                'mag_hard_iron': [round(v, 8) for v in hard_iron.tolist()],
                'mag_soft_iron': [[round(v, 8) for v in row]
                                  for row in soft_iron.tolist()],
            }
            if key == 'phidget':
                entry['mag_north_offset'] = 0.0   # filled in by north phase
            result[key] = entry

        return result

    # ---------------------------------------------------------- north offset

    def _compute_north_offset(self, result):
        """
        Compute heading (rad) when robot faces magnetic north.
        Apply freshly computed phidget calibration to the north samples.
        """
        if len(self.ph_mag_north) < 5:
            print('  Not enough samples — north offset set to 0.0')
            return 0.0

        hi = np.array(result['phidget']['mag_hard_iron'])
        si = np.array(result['phidget']['mag_soft_iron'])
        m  = np.array(self.ph_mag_north)
        corrected = (m - hi) @ si.T
        mean_c = corrected.mean(axis=0)
        # atan2(y, x) gives heading in the sensor's horizontal plane
        return float(np.arctan2(mean_c[1], mean_c[0]))

    # ---------------------------------------------------------- save

    def _save(self, result):
        def fmt3(vals):
            return '[' + ', '.join(f'{v:.8f}' for v in vals) + ']'

        def fmt3x3(rows):
            lines = []
            for row in rows:
                lines.append('    - [' + ', '.join(f'{v:.8f}' for v in row) + ']')
            return '\n' + '\n'.join(lines)

        ts = time.strftime('%Y-%m-%d %H:%M:%S')
        lines = [
            '# IMU Calibration Parameters',
            '# Generated by: ros2 run p3at_robot imu_calibrate.py',
            f'# Generated: {ts}',
            '',
        ]
        for key, name in [('phidget', 'phidget'), ('oak_d', 'oak_d')]:
            d = result[key]
            lines += [
                f'{name}:',
                f'  gyro_bias:     {fmt3(d["gyro_bias"])}',
                f'  accel_bias:    {fmt3(d["accel_bias"])}',
                f'  mag_hard_iron: {fmt3(d["mag_hard_iron"])}',
                f'  mag_soft_iron:{fmt3x3(d["mag_soft_iron"])}',
            ]
            if 'mag_north_offset' in d:
                lines.append(
                    f'  mag_north_offset: {d["mag_north_offset"]:.8f}'
                    f'  # rad — heading when facing magnetic north'
                )
            lines.append('')

        content = '\n'.join(lines)
        paths = [_primary_save_path()]
        # Also write to share dir if it differs (for nodes that are already installed)
        share = os.path.join(_share_config_dir(), 'imu_calibration.yaml')
        if share not in paths:
            paths.append(share)

        print()
        for path in paths:
            try:
                with open(path, 'w') as f:
                    f.write(content)
                print(f'  Saved → {path}')
            except Exception as e:
                print(f'  Failed to save {path}: {e}')

        print('\nCalibration complete.')
        print('Restart phidget_imu.py and oak_camera.py to apply the new values.\n')

    # ---------------------------------------------------------- optional plot

    def _plot(self, result):
        try:
            fig, axes = plt.subplots(2, 1, figsize=(6, 10))
            fig.suptitle('Phidget Magnetometer: raw (top) vs corrected (bottom)')

            pairs = [
                (self.ph_mag, result['phidget'], 'Phidget'),
            ]
            for idx, (mag_l, cal, label) in enumerate(pairs):
                if len(mag_l) < 10:
                    continue
                raw = np.array(mag_l)
                hi  = np.array(cal['mag_hard_iron'])
                si  = np.array(cal['mag_soft_iron'])
                cor = (raw - hi) @ si.T

                axes[0].scatter(raw[:, 0], raw[:, 1], s=1, alpha=0.3)
                axes[0].set_title(f'{label} — raw X-Y')
                axes[0].set_aspect('equal')
                axes[1].scatter(cor[:, 0], cor[:, 1], s=1, alpha=0.3, color='green')
                axes[1].set_title(f'{label} — corrected X-Y')
                axes[1].set_aspect('equal')

            plt.tight_layout()
            out = '/tmp/imu_calibration_plot.png'
            plt.savefig(out, dpi=120)
            print(f'  Plot saved → {out}')
        except Exception as e:
            print(f'  Plot skipped: {e}')


# ------------------------------------------------------------------ entry point

def main(args=None):
    rclpy.init(args=args)
    node = ImuCalibrator()
    cal_thread = threading.Thread(target=node.run, daemon=True)
    cal_thread.start()

    # Spin until calibration finishes
    while not node._done.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
