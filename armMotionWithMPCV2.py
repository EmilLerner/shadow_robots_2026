import time
import numpy as np

from pidController import PIDcontroller
from mpcControllerV6 import MPCController


class ArmMotion():
    def __init__(self):

        # ---- Robot Arm Settings ----
        self.q_real = np.deg2rad([0, -90, 0, -90, 0, 0])

        Kp = np.array([3.0, 3.0, 3.0, 2.2, 1.8, 1.6], dtype=float)
        Ki = np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01], dtype=float)
        Kd = np.array([1.0, 1.0, 1.25, 1.0, 1.0, 1.0], dtype=float)

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

        self.control_count = 0
        self.mpc_update_interval = 10

        self.last_mpc_time = 0.0
        self.last_mpc_cost = None
        self.last_mpc_success = None

    def move_to_base(self, rtde_r, rtde_c):
        print("Moving robot to base position...")
        rtde_c.moveJ(self.q_real.tolist(), speed=0.1, acceleration=1.0)
        print("moveJ has completed")
        self.q_des_prev = np.array(rtde_r.getActualQ(), dtype=float)
        self.last = time.perf_counter()
        print("Base position recorded.")

    def move_with_prediction(self, rtde_r, rtde_c, q_des):
        now = time.perf_counter()
        loop_dt = now - self.last
        self.last = now
        if loop_dt <= 0.0 or loop_dt > 0.2:
            loop_dt = 1 / self.update_rate  # guardrail if we had a hiccup

        loop_dt = max(loop_dt, 1e-4) # prevent excessively low dt
        self.control_count += 1
        # Read robot state
        q = np.array(rtde_r.getActualQ(), dtype=float)
        qd = np.array(rtde_r.getActualQd(), dtype=float)
        q_des = np.array(q_des, dtype=float )

        # Initialise previous target 
        if self.q_des_prev is None: 
            self.q_des_prev = q.copy()

        if (q.shape != (6,) or qd.shape != (6,) or q_des.shape != (6,)):
            raise ValueError("Invalid joint vector shape. " f"q={q.shape}, qd={qd.shape}, " f"q_des={q_des.shape}")

        if not np.all(np.isfinite(q)):
            raise ValueError("Robot joint position contains non-finite values.")
        if not np.all(np.isfinite(qd)):
            raise ValueError("Robot joint velocity contains non-finite values.")
        if not np.all(np.isfinite(q_des)):
            raise ValueError("Desired joint position contains non-finite values.")
        
        # Ignore very small changes 
        dq_des_raw = self.wrap_to_pi(q_des - self.q_des_prev)
        dq_des_raw[np.abs(dq_des_raw) < np.deg2rad(2.0)] = 0.0

        max_step = self.max_joint_speed * loop_dt
        dq_des = np.clip(dq_des_raw, -max_step, max_step)
        q_des_limited = self.wrap_to_pi(self.q_des_prev + dq_des)
        self.q_des_prev = q_des_limited.copy()

        # Optimise mpc-pid gains
        if self.control_count % self.mpc_update_interval == 0: 
            current_gains = { "Kp": np.array(self.pid.Kp, dtype=float).copy(), 
                             "Ki": np.array(self.pid.Ki, dtype=float).copy(), 
                             "Kd": np.array(self.pid.Kd, dtype=float).copy() }
            mpc_start = time.perf_counter()
            try:
                optimized_gains = self.mpc.optimize_gains(q, qd, q_des_limited, current_gains, loop_dt) 
                self.last_mpc_time = (time.perf_counter() - mpc_start)
                valid_gains = True
                for key in ("Kp", "Ki", "Kd"):
                    if key not in optimized_gains:
                        valid_gains = False
                        break
                    values = np.asarray(optimized_gains[key], dtype=float)
                    if values.shape != (6,):
                        valid_gains = False
                        break
                    if not np.all(np.isfinite(values)):
                        valid_gains = False
                        break
                if valid_gains:
                    self.pid.update_gains(optimized_gains["Kp"], optimized_gains["Ki"], optimized_gains["Kd"])
                    print(f"MPC update | " f"time={self.last_mpc_time * 1000:.1f} ms")
                else:
                    print("MPC returned invalid gains. " 
                          "Keeping previous PID gains.")
            except Exception as error:
                self.last_mpc_time = (time.perf_counter() - mpc_start)
                print("MPC optimisation exception: ", error)
                print("Keeping previous PID gains.")

        # calculate joint velocities
        u, e = self.pid.step(q_des_limited, q, qd, loop_dt)
        u = np.asarray(u, dtype=float)
        e = np.asarray(e, dtype=float)

        # Safety: if comms or state look weird, bail
        if not np.all(np.isfinite(u)) or not np.all(np.isfinite(e)):
            print("Non-finite in control; stopping.")
            try:
                rtde_c.speedStop()
            except Exception:
                pass
            return u, e

        # Limit joint velocity
        u = np.clip( u, -self.max_joint_speed, self.max_joint_speed)

        # Send velocity commands to UR5
        rtde_c.speedJ(u.tolist(), self.max_joint_acceleration, loop_dt) 
        return u, e

    def wrap_to_pi(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

