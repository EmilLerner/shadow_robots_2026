import numpy as np
from scipy.optimize import minimize
from pidController import PIDcontroller

class MPCController():
    def __init__(self, pred_horizon=20, n_joints=6):
        self.N = pred_horizon
        self.n_joints = n_joints

        # Cost Matrix Weights
        self.W_error = 100.0
        self.W_smooth = 1.0

    def predict_system_dynamics(self, q, q_dot, control_velocity, dt):
        predicted_q = q + q_dot * dt + 0.5 * (control_velocity - q_dot) * dt
        predicted_q_dot = control_velocity
        return predicted_q, predicted_q_dot

    def cost_function(self, gains_vector, current_q, current_q_dot, target_q, prev_gains, dt):
        Kp = gains_vector[0:6]
        Ki = gains_vector[6:12]
        Kd = gains_vector[12:18]

        total_cost = 0.0

        q_sim = np.copy(current_q)
        q_dot_sim = np.copy(current_q_dot)
        integral_sim = np.zeros(self.n_joints)

        for _ in range(self.N):
            error = target_q - q_sim
            integral_sim += error * dt
            u_vel = (Kp * error) + (Ki * integral_sim) + (Kd * q_dot_sim)

            q_sim, q_dot_sim = self.predict_system_dynamics(q_sim, q_dot_sim, u_vel, dt)

            total_cost += self.W_error * np.sum(np.square(target_q - q_sim))

        total_cost += self.W_smooth * np.sum(np.square(gains_vector - prev_gains))

        return total_cost

    def optimize_gains(self, current_q, current_q_dot, target_q, current_gains, dt):
        prev_gains = np.concatenate([current_gains['Kp'], current_gains['Ki'], current_gains['Kd']])

        bounds_Kp = [(1.0, 25.0)] * 6
        bounds_Ki = [(0.0, 2.0)] * 6
        bounds_Kd = [(0.0, 1.5)] * 6
        bounds = bounds_Kp + bounds_Ki + bounds_Kd

        res = minimize(self.cost_function,
                       prev_gains,
                       args=(current_q, current_q_dot, target_q, prev_gains, dt),
                       method='SLSQP',
                       bounds=bounds,
                       options={'maxiter': 10, 'disp': False}
                       )

        optimized_gains = {
            'Kp': res.x[0:6],
            'Ki': res.x[6:12],
            'Kd': res.x[12:18]
        }

        return optimized_gains
