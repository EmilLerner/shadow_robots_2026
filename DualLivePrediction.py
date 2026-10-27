"""Live Xsens + RF + offline UR5 IK/FK evaluation. No CSV or robot connection.

Requires the project's existing modules. Robot FK adapter assumes
forward_kinematics(q, a, d, b, tp); adjust robot_tcp() if yours differs.
"""
from collections import deque
import time
import numpy as np
import pandas as pd
import joblib
import xsensdeviceapi as xda
import DualVariableConfig as cfg
from armMotionV2 import ArmMotion
from mtwManipulator import MtwManipulator
from wirelessMasterCallback import WirelessMasterCallback
from inverse_kinematics_solver5 import choose_best_ik, forward_kinematics

SENSOR_IDS = {
    '00B4F1B1': 'S1', '00B4F1AE': 'S2', '00B4F12E': 'S3',
    '00B4F12F': 'S4', '00B4F1B3': 'S5', '00B4F130': 'S6'
}
SENSORS = tuple(f'S{i}' for i in range(1, 7))
PARTS = ('Right elbow', 'Right wrist', 'Right hand',
         'Left elbow', 'Left wrist', 'Left hand')


def column_map():
    result = {}
    for i, sensor in enumerate(SENSORS):
        side = 'R' if i < 3 else 'L'
        result[sensor] = ([f'{sensor}_ACC_{axis}_{side}' for axis in 'XYZ']
                          + [f'{sensor}_GYRO_{axis}_{side}' for axis in 'XYZ']
                          + [f'{sensor}_QUAT_{axis}_{side}' for axis in 'WXYZ'])
    return result


COLUMNS = column_map()


class XsensCollector:
    """Produce latest complete six-sensor rows using the existing MTW classes."""
    def __init__(self, update_rate=80, radio_channel=19):
        self.update_rate = update_rate
        self.radio_channel = radio_channel
        self.control = None
        self.manipulator = None
        self.data = {s: None for s in SENSORS}
        self.updated = {s: False for s in SENSORS}
        self.packet_counts = {s: 0 for s in SENSORS}
        self.sample_count = 0
        self.start_time = None

    def start(self):
        self.control = xda.XsControl.construct()
        callback = WirelessMasterCallback()
        self.manipulator = MtwManipulator(
            self.control, callback, self.update_rate,
            self.radio_channel, SENSOR_IDS
        )
        print('Scanning for Xsens devices...')
        self.manipulator.scanXsDevices()
        self.manipulator.startConfigMode()
        print('Waiting for MTW connections...')
        self.manipulator.waitForConnections()
        self.manipulator.startMeasurement()
        self.manipulator.resetOrientationData()
        self.manipulator.startRecording()
        self.start_time = time.perf_counter()

    def poll(self):
        if self.manipulator is None:
            raise RuntimeError('Xsens collector not started')
        for callback in self.manipulator.getMtwCallbacks():
            sensor_id = str(callback.device().deviceId())
            sensor = SENSOR_IDS.get(sensor_id)
            while callback.dataAvailable():
                packet = callback.getOldestPacket()
                try:
                    if sensor is None:
                        continue
                    acc = packet.calibratedAcceleration()
                    gyro = packet.calibratedGyroscopeData()
                    quat = packet.orientationQuaternion()
                    self.data[sensor] = ([float(acc[i]) for i in range(3)]
                                         + [float(gyro[i]) for i in range(3)]
                                         + [float(quat[i]) for i in range(4)])
                    self.updated[sensor] = True
                    self.packet_counts[sensor] += 1
                finally:
                    callback.deleteOldestPacket()
        if not all(self.updated.values()):
            return None
        row = {'timestamp': time.perf_counter() - self.start_time}
        for sensor in SENSORS:
            row.update(zip(COLUMNS[sensor], self.data[sensor]))
        self.updated = {s: False for s in SENSORS}
        self.sample_count += 1
        return row

    def stop(self):
        if self.manipulator is not None:
            try:
                self.manipulator.stopRecording()
            except Exception as exc:
                print(f'Warning stopping recording: {exc}')
            # The project MtwManipulator does not implement stopMeasurement().
            self.manipulator = None


def robot_tcp(q, a, d, b, tp):
    """Call EXISTING FK; accept XYZ, 4x4 transform, or (position, rotation).

    If your forward_kinematics signature differs, modify this adapter only.
    """
    result = forward_kinematics(q, a, d, b, tp)
    if isinstance(result, (tuple, list)) and len(result) == 2:
        result = result[0]
    result = np.asarray(result, dtype=float)
    if result.shape == (4, 4):
        return result[:3, 3].copy()
    if result.shape == (3,):
        return result.copy()
    raise ValueError(f'Unexpected forward_kinematics output shape: {result.shape}')


class LivePrediction:
    """Keep only pending predictions; aggregate errors without saving samples."""
    def __init__(self, buffer_limit=32):
        self.features = list(cfg.IMU_FEATURES)
        self.targets = list(cfg.TARGET_FEATURES)
        if len(self.targets) != 18:
            raise ValueError('TARGET_FEATURES must have 18 values (6 XYZ positions)')
        self.model = joblib.load(cfg.COMBINED_MODEL)
        dims = [cfg.UPPER_ARM_LENGTH, cfg.FOREARM_LENGTH, cfg.HAND_LENGTH]
        self.right_arm = ArmMotion(dims, 'NONE', devices={})
        self.left_arm = ArmMotion(dims, 'INV', devices={})
        self.horizon = float(cfg.SAMPLE_SHIFT) * float(cfg.SAMPLE_TIME)
        if self.horizon <= 0:
            raise ValueError('Prediction horizon must be positive')
        self.human_pending = deque()
        self.robot_pending = deque()
        self.buffer_limit = int(buffer_limit)
        self.previous_actual = None
        self.count_human = 0
        self.human_sum = np.zeros(6)
        self.human_sq = np.zeros(6)
        self.count_robot = np.zeros(2, dtype=int)
        self.robot_xyz_sum = np.zeros(2)
        self.robot_xyz_sq = np.zeros(2)
        self.robot_joint_sum = np.zeros(2)
        self.robot_joint_sq = np.zeros(2)
        self.failed_ik = 0
        self.dropped = 0
        self.a = cfg.ROBOT_A
        self.d = cfg.ROBOT_D
        self.b = cfg.ROBOT_B
        self.tp = cfg.ROBOT_TP
        # Reference angles select an IK branch, not measured robot angles.
        self.q_reference = [np.zeros(6), np.zeros(6)]
        self.rotations = [self._fixed_rotation(self.right_arm),
                          self._fixed_rotation(self.left_arm)]

    @staticmethod
    def _fixed_rotation(arm):
        # Use the orientation convention in the existing ArmMotion module.
        rotation = np.asarray(
            arm.gen_rot_mat([0, 0, 0, 1], arm.alpha, arm.beta, arm.gamma),
            dtype=float
        )
        if arm.orientation == 'INV':
            rotation = np.diag([-1.0, -1.0, 1.0]) @ rotation
        return rotation

    def human_xyz(self, row):
        quats = {
            name: tuple(float(row[col]) for col in columns)
            for name, columns in cfg.QUAT_COLUMNS.items()
        }
        right = {'SHOU': quats['RSHOU'], 'fARM': quats['RfARM'],
                 'HAND': quats['RHAND']}
        left = {'SHOU': quats['LSHOU'], 'fARM': quats['LfARM'],
                'HAND': quats['LHAND']}
        return np.concatenate((
            self.right_arm.calculate_human_positions(right),
            self.left_arm.calculate_human_positions(left)
        )).astype(float)

    def robot_from_human(self, human_xyz):
        """Return (2,6) IK joint angles and (2,3) FK TCP positions.

        Only the hand XYZ (indices 6:9 and 15:18) defines the IK target.
        The elbow/wrist XYZ are evaluated separately as human predictions.
        """
        q_all, tcp_all = [], []
        for arm_index, hand in enumerate((human_xyz[6:9], human_xyz[15:18])):
            q = choose_best_ik(self.q_reference[arm_index], hand,
                               self.rotations[arm_index],
                               self.a, self.d, self.b, self.tp)
            if q is None:
                raise ValueError('IK returned no solution')
            q = np.asarray(q, dtype=float).reshape(6)
            if not np.all(np.isfinite(q)):
                raise ValueError('IK returned nonfinite angles')
            tcp = robot_tcp(q, self.a, self.d, self.b, self.tp)
            q_all.append(q)
            tcp_all.append(tcp)
        return np.stack(q_all), np.stack(tcp_all)

    def process(self, row):
        """Predict now, compare earlier predictions when target time passes.

        Comparisons use the first complete sample at/after target time.
        Host timestamps approximate aligned packet time; not device time.
        """
        t = float(row['timestamp'])
        actual = self.human_xyz(row)
        events = []
        while self.human_pending and self.human_pending[0][0] <= t:
            target, predicted = self.human_pending.popleft()
            error_cm = np.linalg.norm((predicted - actual).reshape(6, 3), axis=1) * 100
            self.human_sum += error_cm
            self.human_sq += error_cm ** 2
            self.count_human += 1
            events.append(('human', t - target, error_cm))

        while self.robot_pending and self.robot_pending[0][0] <= t:
            target, q_pred, tcp_pred = self.robot_pending.popleft()
            try:
                q_actual, tcp_actual = self.robot_from_human(actual)
            except (ValueError, TypeError, ArithmeticError) as exc:
                self.failed_ik += 1
                events.append(('ik_failed', str(exc)))
                continue
            tcp_err = np.linalg.norm(tcp_pred - tcp_actual, axis=1) * 100
            # Shortest signed angular difference, in degrees.
            delta = np.arctan2(np.sin(q_pred - q_actual),
                               np.cos(q_pred - q_actual))
            joint_err = np.sqrt(np.mean(np.rad2deg(delta) ** 2, axis=1))
            self.count_robot += 1
            self.robot_xyz_sum += tcp_err
            self.robot_xyz_sq += tcp_err ** 2
            self.robot_joint_sum += joint_err
            self.robot_joint_sq += joint_err ** 2
            events.append(('robot', t - target, tcp_err, joint_err))

        x = pd.DataFrame([[float(row[name]) for name in self.features]],
                         columns=self.features)
        predicted = np.asarray(self.model.predict(x)[0], dtype=float)
        if predicted.shape != (18,):
            raise ValueError(f'Expected 18 RF outputs, got {predicted.shape}')
        target = t + self.horizon
        self.human_pending.append((target, predicted.copy()))
        try:
            q_pred, tcp_pred = self.robot_from_human(predicted)
            self.robot_pending.append((target, q_pred.copy(), tcp_pred.copy()))
        except (ValueError, TypeError, ArithmeticError) as exc:
            self.failed_ik += 1
            events.append(('ik_failed', str(exc)))
        # Bound both buffers; overflow is counted, not silently ignored.
        for queue in (self.human_pending, self.robot_pending):
            while len(queue) > self.buffer_limit:
                queue.popleft()
                self.dropped += 1
        return events

    def summary(self):
        lines = [f'Human comparisons: {self.count_human}',
                 f'Robot comparisons (right/left): {self.count_robot.tolist()}',
                 f'IK failures: {self.failed_ik} | Buffer drops: {self.dropped}']
        if self.count_human:
            lines.append('Human 3D mean error / RMSE (cm):')
            for name, mean, sq in zip(PARTS, self.human_sum / self.count_human,
                                      np.sqrt(self.human_sq / self.count_human)):
                lines.append(f'  {name:13s}: {mean:7.2f} / {sq:7.2f}')
        for i, side in enumerate(('Right', 'Left')):
            if self.count_robot[i]:
                n = self.count_robot[i]
                lines.append(
                    f'{side} robot FK TCP mean/RMSE: '
                    f'{self.robot_xyz_sum[i]/n:.2f}/{np.sqrt(self.robot_xyz_sq[i]/n):.2f} cm; '
                    f'joint RMS difference mean/RMSE: '
                    f'{self.robot_joint_sum[i]/n:.2f}/{np.sqrt(self.robot_joint_sq[i]/n):.2f} deg'
                )
        return '\n'.join(lines)
