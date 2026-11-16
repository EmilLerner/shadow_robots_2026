import numpy as np
import pandas as pd

df = pd.read_csv("joint_angles_MPC_NONE.csv")

# Actual and desired joint positions, in radians
q = df[[f"q{i}" for i in range(1, 7)]].to_numpy()
q_des = df[[f"q_des{i}" for i in range(1, 7)]].to_numpy()

error = q_des - q

error = (error + np.pi) % (2 * np.pi) - np.pi

error_deg = np.rad2deg(error)

# RMSE for each joint
rmse_deg = np.sqrt(np.mean(error_deg**2, axis=0))

# Maximum absolute error for each joint
max_error_deg = np.max(np.abs(error_deg), axis=0)

print("RMSE per joint (degrees):")
print(np.round(rmse_deg, 3))

print("Maximum error per joint (degrees):")
print(np.round(max_error_deg, 3))

# Control effort
u = df[[f"u_lim{i}" for i in range(1, 7)]].to_numpy()
t = df["time"].to_numpy()

# Remove invalid rows
valid = np.isfinite(t) & np.all(np.isfinite(u), axis=1)
t = t[valid]
u = u[valid]

if len(t) > 1:
    dt = np.diff(t)
    valid_dt = dt > 0

    duration = np.sum(dt[valid_dt])

    # Time-averaged squared velocity command
    effort = np.sum(
        0.5 * (
            np.sum(u[:-1][valid_dt]**2, axis=1)
            + np.sum(u[1:][valid_dt]**2, axis=1)
        ) * dt[valid_dt]
    ) / duration

    rms_velocity = np.sqrt(effort)

    print("RMS commanded joint velocity (rad/s):")
    print(round(rms_velocity, 4))

    print("Integrated squared control effort (rad²/s):")
    print(round(effort * duration, 4))

# MPC computation time
if "mpc_ms" in df.columns:
    mpc_times = df["mpc_ms"].dropna()
    if len(mpc_times):
        print("MPC computation time (ms):")
        print("Mean:", round(mpc_times.mean(), 3))
        print("Median:", round(mpc_times.median(), 3))
        print("95th percentile:", round(mpc_times.quantile(0.95), 3))
        print("Maximum:", round(mpc_times.max(), 3))
