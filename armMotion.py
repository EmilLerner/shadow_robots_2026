import time

import numpy as np
import inverse_kinematics_solver4 as iks

from math import pi

import xsensdeviceapi as xda
from scipy.spatial.transform import Rotation as R

from pidController import PIDcontroller
import csv

class ArmMotion():
    def __init__(self, arms_dim, orientation, devices):
        super().__init__()

        # ---- Robot Arm Settings ----
        
        self.a = [0.0, 0.4251, 0.39215, 0.0, 0.0, 0.0]  
        self.d = [0.0, 0.0, 0.0, 0.11000, 0.09475, 0.0]
        self.b = 0.0892 
        self.tp = 0.07495

        self.q_real = np.deg2rad([0, -90, 0, -90, 0, 0])

        self.orientation = orientation

        Kp = [3.0, 3.0, 3.0, 2.2, 1.8, 1.6]
        Ki = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
        Kd = [1.0, 1.0, 1.25, 1.0, 1.0, 1.0]

        self.u_max = 10.0
        self.i_max = 0.6
        self.backcalc_beta = 0.15

        self.pid = PIDcontroller(Kp=Kp, Ki=Ki, Kd=Kd, u_max=self.u_max, i_max=self.i_max, backcalc_beta=self.backcalc_beta)

        self.update_rate = 80
        self.step = 2.0
        self.max_joint_speed = 2.0
        self.max_joint_acceleration = 2.0

        self.init_q = [0, 0, 0, 0, 0, 0]

        # ---- Human Arm Settings ---- 

        self.L_upper   = arms_dim[0]  # shoulder → elbow
        self.L_forearm = arms_dim[1]  # elbow → wrist
        self.L_hand    = arms_dim[2]  # wrist → hand center

        self.local_upper    = np.array([0.0, self.L_upper, 0.0])
        self.local_forearm  = np.array([0.0, self.L_forearm, 0.0])
        self.local_hand     = np.array([0.0, self.L_hand, 0.0])

        self.shoulder_pos = np.array([0.0, 0.0, 0.0])

        self.alpha  = np.deg2rad(-180)
        self.beta   = np.deg2rad(0)
        self.gamma  = np.deg2rad(-90)

        self.shoulder_q = R.identity()
        self.elbow_q    = R.identity()
        self.wrist_q    = R.identity()

        self.deviceID_to_bodypart = devices

        self.last = time.perf_counter()
        self.q_des_prev = None
        self.hand_pos_prev = 0


        self.csv_file = "joint_angles_" + orientation + ".csv"

        with open(self.csv_file, "w", newline="") as f:
            writer = csv.writer(f)
            header = ["time"]
            header += [f"curr{i+1}" for i in range(3)]
            header += [f"des{i+1}" for i in range(3)]
            writer.writerow(header)


    def move_to_base(self, rtde_r, rtde_c):
        self.q_des_prev = np.array(rtde_r.getActualQ())

        elbow_pos = self.shoulder_pos + self.local_upper
        wrist_pos = elbow_pos + self.local_forearm
        hand_pos = wrist_pos + self.local_hand

        hand_pos[2] += 0.1

        rot_matrix = self.gen_rot_mat([0,0,0,1], self.alpha, self.beta, self.gamma)

        if self.orientation == "INV":
            R_flip = np.diag([-1.0, -1.0, 1.0])
            hand_pos = R_flip @ hand_pos
            rot_matrix = R_flip @ rot_matrix

        print(hand_pos)

        print("Performing moveJ to an intial instance")
        q_current = np.array(rtde_r.getActualQ())
        q_des = iks.choose_best_ik(q_current, hand_pos, rot_matrix, self.a, self.d, self.b, self.tp) + self.q_real

        q_des = self.wrap_to_2pi(q_des)

        print(q_des)

        rtde_c.moveJ(q_des, speed=0.1, acceleration=1.0)
        print("moveJ has completed")
        self.q_des_prev = np.array(rtde_r.getActualQ())
        self.init_q = np.array(rtde_r.getActualQ())
        self.hand_pos_prev = hand_pos

    def move_with_sensors(self, rtde_r, rtde_c, quat_data):
        # parse shoulder orientation
        shou_qW, shou_qX, shou_qY, shou_qZ = quat_data.get("SHOU")
        shoulder_q = R.from_quat([-shou_qY, shou_qX, shou_qZ, shou_qW])
        # parse forearm orientation
        elbow_qW, elbow_qX, elbow_qY, elbow_qZ = quat_data.get("fARM")
        elbow_q = R.from_quat([-elbow_qY, elbow_qX, elbow_qZ, elbow_qW])
        # parse hand orientation
        wrist_qW, wrist_qX, wrist_qY, wrist_qZ = quat_data.get("HAND")
        wrist_q = R.from_quat([-wrist_qY, wrist_qX, wrist_qZ, wrist_qW])
    
        
        rot_matrix = self.gen_rot_mat([wrist_qX, wrist_qY, wrist_qZ, wrist_qW], self.alpha, self.beta, self.gamma)

        elbow_pos = self.shoulder_pos + shoulder_q.apply(self.local_upper)
        wrist_pos = elbow_pos + elbow_q.apply(self.local_forearm)
        hand_pos = wrist_pos + wrist_q.apply(self.local_hand)

        hand_pos[2] += 0.1 # Elevate hand pos slightly in z-direction

        now = time.perf_counter()
        loop_dt = now - self.last
        self.last = now
        if loop_dt <= 0.0 or loop_dt > 0.2:
            loop_dt = 1/self.update_rate  # guardrail if we had a hiccup

        # Read robot state
        q = np.array(rtde_r.getActualQ(), dtype=float)
        qd = np.array(rtde_r.getActualQd(), dtype=float)

        if self.orientation == "INV":
            R_flip = np.diag([-1.0, -1.0, 1.0])
            hand_pos = R_flip @ hand_pos
            rot_matrix = R_flip @ rot_matrix

        self.hand_pos_prev = hand_pos

        q_current = np.array(rtde_r.getActualQ())
        q_des = iks.choose_best_ik(q_current, hand_pos, rot_matrix, self.a, self.d, self.b, self.tp) + self.q_real
        
        q_current_real = q_current - self.q_real
        current_pos = iks.forward_kinematics(q_current_real, self.a, self.d, self.b, self.tp)

        row = [time.time()] + current_pos + hand_pos.tolist()

        # with open(self.csv_file, "a", newline="") as f:
        #     writer = csv.writer(f)
        #     writer.writerow(row)
        
        
        q_des = np.array(q_des, dtype=float)
        q_des = self.wrap_to_2pi(q_des)

        dq_des_raw = self.wrap_to_pi(q_des - self.q_des_prev)

        dq_des_raw[np.abs(dq_des_raw) < np.deg2rad(2.0)] = 0.0
        max_step = self.step * loop_dt
        dq_des = np.clip(dq_des_raw, -max_step, max_step)
        q_des = self.wrap_to_pi(self.q_des_prev + dq_des)
        self.q_des_prev = q_des.copy()

        u, e = self.pid.step(q_des, q, qd, loop_dt)

        # Safety: if comms or state look weird, bail
        if not np.all(np.isfinite(u)) or not np.all(np.isfinite(e)):
            print("Non-finite in control; stopping.")
            rtde_c.speedStop()
    
        u_limited = np.clip(u, -self.max_joint_speed, self.max_joint_speed)

        Kp_sched, Ki_sched, Kd_sched = self.gain_scheduling(u_limited, self.max_joint_speed)
        self.pid.update_gains(Kp_sched, Ki_sched, Kd_sched)
        rtde_c.speedJ(u_limited.tolist(), self.max_joint_acceleration, 1/self.update_rate)

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
    
    
