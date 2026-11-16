import numpy as np
import time
from scipy.optimize import minimize
from pidController import PIDcontroller

class MPCController():
    def __init__(self, pred_horizon=20, n_joints=6):
        self.N = pred_horizon
        self.n_joints = n_joints

        # Cost Matrix Weights
        self.W_error = 5.0
        self.W_smooth = 1.0
        self.W_velocity = 2.0
        self.W_accel = 0.2

        self.max_joint_speed = 2.0
        self.max_joint_acceleration = 2.0

    def predict_system_dynamics(self, q, q_dot, control_velocity, dt):
        acceleration = (control_velocity - q_dot) / dt
        acceleration = np.clip(acceleration, -self.max_joint_acceleration, self.max_joint_acceleration)
        predicted_q_dot = q_dot + acceleration * dt
        predicted_q_dot = np.clip(predicted_q_dot, -self.max_joint_speed, self.max_joint_speed)
        predicted_q = q + 0.5 * (q_dot + predicted_q_dot) * dt
        return predicted_q, predicted_q_dot, acceleration

    def cost_function(self, gains_vector, current_q, current_q_dot, target_q, prev_gains, dt):
        Kp = gains_vector[0:6]
        Ki = gains_vector[6:12]
        Kd = gains_vector[12:18]

        total_cost = 0.0

        q_sim = np.copy(current_q)
        q_dot_sim = np.copy(current_q_dot)
        integral_sim = np.zeros(self.n_joints)

        previous_vel = q_dot_sim.copy()

        for _ in range(self.N):
            error = target_q - q_sim
            integral_sim += error * dt
            u_vel = (Kp * error) + (Ki * integral_sim) - (Kd * q_dot_sim)

            u_vel = np.clip(u_vel, -self.max_joint_speed, self.max_joint_speed)

            q_next, q_dot_next, accel = self.predict_system_dynamics(q_sim, q_dot_sim, u_vel, dt)

            position_error = target_q - q_next

            delta_velocity = q_dot_next - previous_vel

            total_cost += (self.W_error * np.sum(np.square(position_error)) + self.W_velocity * np.sum(np.square(u_vel)) + self.W_accel * np.sum(np.square(delta_velocity)))

            q_sim = q_next
            q_dot_sim = q_dot_next

            previous_vel = q_dot_next.copy()

        total_cost += self.W_smooth * np.sum(np.square(gains_vector - prev_gains))

        return total_cost

    def optimize_gains(self, current_q, current_q_dot, target_q, current_gains, dt, joint_history=None):
        if joint_history is not None:
            assert joint_history.shape == (self.N + 1, self.n_joints), (
                f"Expected joint_history shape {(self.N + 1, self.n_joints)}, "
                f"got {joint_history.shape}"
            )
            current_q = joint_history[0]
            target_q = joint_history[-1]
        
        prev_gains = np.concatenate([current_gains['Kp'], current_gains['Ki'], current_gains['Kd']])

        bounds_Kp = [(1.8, 3.0)] * 6
        bounds_Ki = [(0.01, 0.02)] * 6
        bounds_Kd = [(0.5, 1.5)] * 6
        bounds = bounds_Kp + bounds_Ki + bounds_Kd

        start = time.perf_counter()

        res = minimize(
            self.cost_function,
            prev_gains,
            args=(current_q, current_q_dot, target_q, prev_gains, dt),
            method='SLSQP',
            bounds=bounds,
            options={'maxiter': 20, 'ftol': 1e-2, 'disp': False}
        )

        elapsed = time.perf_counter() - start

        # print(f"MPC: success={res.success}, " f"iterations={res.nit}, " f"cost={res.fun:.2f}, " f"time={elapsed*1000:.1f} ms")

        # Check optimisation result
        if not res.success:
            # print("MPC optimisation warning:", res.message, "| iterations:", res.nit, "| cost:", res.fun, "| time:")
            return {"Kp": current_gains["Kp"].copy(), "Ki": current_gains["Ki"].copy(), "Kd": current_gains["Kd"].copy()}


        optimized_gains = {
            'Kp': res.x[0:6],
            'Ki': res.x[6:12],
            'Kd': res.x[12:18]
        }

        # print("Initial gains:", prev_gains)
        # print("Optimized gains:", res.x)

        return optimized_gains

