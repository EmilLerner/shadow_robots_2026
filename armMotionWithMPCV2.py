import time
import numpy as np

from pidController import PIDcontroller
from mpcControllerV5 import MPCController


class ArmMotion():
    def __init__(self):

        # ---- Robot Arm Settings ----
        self.q_real = np.deg2rad([0, -90, 0, -90, 0, 0])

        self.orientation = orientation

        Kp = [3.0, 3.0, 3.0, 2.2, 1.8, 1.6]
        Ki = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
        Kd = [1.0, 1.0, 1.25, 1.0, 1.0, 1.0]

        self.u_max = 10.0
        self.i_max = 0.6
        self.backcalc_beta = 0.15
        self.pred_horizon = 20

        
        # PID settings 
        self.pid = PIDcontroller(Kp=Kp, Ki=Ki, Kd=Kd, u_max=self.u_max, i_max=self.i_max, backcalc_beta=self.backcalc_beta)

        # MPC settings
        self.mpc = MPCController(pred_horizon=self.pred_horizon)

        # Control settings
        self.update_rate = 80
        self.max_joint_speed = 2.0
        self.max_joint_acceleration = 2.0
        self.last = time.perf_counter()
        self.q_des_prev = None

    def move_to_base(self, rtde_r, rtde_c):

        print("Moving robot to base position...")
        rtde_c.moveJ(q_des, speed=0.1, acceleration=1.0)
        print("moveJ has completed")
        self.q_des_prev = np.array(rtde_r.getActualQ(), dtype=float)

    def move_with_prediction(self, rtde_r, rtde_c, q_des):

        now = time.perf_counter()
        loop_dt = now - self.last
        self.last = now
        if loop_dt <= 0.0 or loop_dt > 0.2:
            loop_dt = 1 / self.update_rate  # guardrail if we had a hiccup

        # Read robot state
        q = np.array(rtde_r.getActualQ(), dtype=float)
        qd = np.array(rtde_r.getActualQd(), dtype=float)
        q_des = np.array(q_des, dtype=float )

        # Initialise previous target 
        if self.q_des_prev is None: 
            self.q_des_prev = q.copy()

        # Ignore very small changes 
        dq_des_raw = self.wrap_to_pi(q_des - self.q_des_prev)
        dq_des_raw[np.abs(dq_des_raw) < np.deg2rad(2.0)] = 0.0

        max_step = self.step * loop_dt
        dq_des = np.clip(dq_des_raw, -max_step, max_step)
        q_des_limited = self.wrap_to_pi(self.q_des_prev + dq_des)
        self.q_des_prev = q_des_limited.copy()

        # Optimise mpc-pid gains
        current_gains = { "Kp": self.pid.Kp, "Ki": self.pid.Ki, "Kd": self.pid.Kd } 
        optimized_gains = self.mpc.optimize_gains( q, qd, q_des_limited, current_gains, loop_dt) 
        self.pid.update_gains(optimized_gains["Kp"], optimized_gains["Ki"], optimized_gains["Kd"])\

        # calculate joint velocities
        u, e = self.pid.step( q_des_limited, q, qd, loop_dt)

        # Safety: if comms or state look weird, bail
        if not np.all(np.isfinite(u)) or not np.all(np.isfinite(e)):
            print("Non-finite in control; stopping.")
            rtde_c.speedStop()
            return u, e

        # Limit joint velocity
        u = np.clip( u, -self.max_joint_speed, self.max_joint_speed)

        # Send velocity commands to UR5
        rtde_c.speedJ(u.tolist(), self.max_joint_acceleration, loop_dt) 
        return u, e

        def wrap_to_pi(self, angle): 
            return ((angle + np.pi) % (2 * np.pi)) - np.pi



    def ema_pos(self, prev_pos, curr_pos, alpha=0.2):
        return alpha * curr_pos + (1 - alpha) * prev_pos

    def limit_angles(self, angles, u_limited):
        limits = np.deg2rad([45, 30, 90, 90, 90, 180])
        print(np.rad2deg(angles))
        angles = np.array(angles)
        u_limited = np.array(u_limited)
        initial_angles = np.array(self.init_q)

        for i in range(6):
            upper = initial_angles[i] + limits[i]
            lower = initial_angles[i] - limits[i]
            if angles[i] > upper or angles[i] < lower:
                u_limited[i] = 0.0  # stop motion if outside limit

        return u_limited

    def wrap_to_2pi(self, angle):
        return (angle - 2 * np.pi) % (4 * np.pi) - 2 * np.pi

    def wrap_to_pi(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def gen_rot_mat(self, angles, alpha, beta, gamma):
        angle_X = angles[0]
        angle_Y = angles[1]
        angle_Z = angles[2]
        angle_W = angles[3]

        R_sensor = R.from_quat([-angle_Y, angle_X, angle_Z, angle_W]).as_matrix()
        rot_matrix = R_sensor @ R.from_euler('zxy', [beta, gamma, alpha]).as_matrix()
        return rot_matrix

    def gain_scheduling(self, current_joint_velocity, max_joint_velocity):
        # Base gains
        Kp_base = [1.8, 1.8, 1.8, 1.8, 1.8, 1.8]
        Ki_base = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
        Kd_base = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]

        Kp_max = [3.0, 3.0, 3.0, 3.0, 3.0, 3.0]
        Ki_max = [0.02, 0.02, 0.02, 0.02, 0.02, 0.02]
        Kd_max = [1.0, 1.0, 1.5, 1.0, 1.0, 1.0]

        Kp = []
        Ki = []
        Kd = []
        for i in range(6):
            v = abs(current_joint_velocity[i])
            ratio = min(v / max_joint_velocity, 1.0)  # Clamp to 1.0
            Kp.append(Kp_base[i] + (Kp_max[i] - Kp_base[i]) * ratio)
            Ki.append(Ki_base[i] + (Ki_max[i] - Ki_base[i]) * ratio)
            Kd.append(Kd_base[i] + (Kd_max[i] - Kd_base[i]) * ratio)
        return Kp, Ki, Kd

