import matplotlib.pyplot as plt
import numpy as np

# Data definitions
joints = [f'Joint {i}' for i in range(1, 7)]
x = np.arange(len(joints))
width = 0.35

mpc_rmse = [11.953, 11.346, 6.542, 11.636, 3.049, 15.698]
pid_rmse = [13.488, 13.947, 7.467, 14.935, 3.249, 25.468]

mpc_max = [56.648, 49.057, 34.288, 51.847, 17.891, 67.464]
pid_max = [53.204, 59.447, 36.831, 66.822, 16.618, 90.685]

# --- Figure 1: RMSE & Max Error ---
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

# RMSE Plot
ax1.bar(x - width/2, mpc_rmse, width, label='MPC-PID', color='#1f77b4')
ax1.bar(x + width/2, pid_rmse, width, label='PID', color='#ff7f0e')
ax1.set_title('Average RMSE per Joint')
ax1.set_ylabel('Degrees')
ax1.set_xticks(x)
ax1.set_xticklabels(joints)
ax1.legend()
ax1.grid(axis='y', linestyle='--', alpha=0.7)

# Max Error Plot
ax2.bar(x - width/2, mpc_max, width, label='MPC-PID', color='#1f77b4')
ax2.bar(x + width/2, pid_max, width, label='PID', color='#ff7f0e')
ax2.set_title('Average Maximum Error per Joint')
ax2.set_ylabel('Degrees')
ax2.set_xticks(x)
ax2.set_xticklabels(joints)
ax2.legend()
ax2.grid(axis='y', linestyle='--', alpha=0.7)

plt.tight_layout()
plt.show()

# --- Figure 2: Velocity and Control Effort ---
# Data
labels = ['MPC-PID', 'PID']
velocity = [0.8218, 0.9433]         # rad/s
control_effort = [24.7851, 31.4323]  # rad²/s

x = np.arange(len(labels))
width = 0.35  # Width of the bars

fig, ax1 = plt.subplots(figsize=(7, 5))

# Primary Axis: Velocity
color = '#1f77b4'
ax1.set_xlabel('Control Strategy', fontweight='bold', labelpad=10)
ax1.set_ylabel('Avg RMS Commanded Joint Velocity (rad/s)', color=color, fontweight='bold')
bars1 = ax1.bar(x - width/2, velocity, width, color=color, label='Joint Velocity')
ax1.tick_params(axis='y', labelcolor=color)
ax1.set_ylim(0, 1.2)

# Secondary Axis: Control Effort
ax2 = ax1.twinx()  
color = '#ff7f0e'
ax2.set_ylabel('Avg Integrated Squared Control Effort (rad²/s)', color=color, fontweight='bold')
bars2 = ax2.bar(x + width/2, control_effort, width, color=color, label='Control Effort')
ax2.tick_params(axis='y', labelcolor=color)
ax2.set_ylim(0, 40)

# Formatting
plt.xticks(x, labels)
ax1.set_title('Controller Performance Comparison', fontweight='bold', fontsize=14, pad=15)
ax1.grid(True, axis='y', linestyle='--', alpha=0.3)

# Value labels on top of bars
def autolabel(bars, ax, precision=3):
    for bar in bars:
        height = bar.get_height()
        ax.annotate(f'{height:.{precision}f}',
                    xy=(bar.get_x() + bar.get_width() / 2, height),
                    xytext=(0, 3),  # 3 points vertical offset
                    textcoords="offset points",
                    ha='center', va='bottom', fontsize=9)

autolabel(bars1, ax1, 4)
autolabel(bars2, ax2, 2)

fig.tight_layout()
plt.show()

# --- Figure 3: MPC Computation Latency ---
latency_metrics = ['Median', 'Mean', '95th Percentile', 'Maximum']
latency_values = [158.187, 170.320, 336.023, 447.453]

plt.figure(figsize=(8, 4))
bars = plt.barh(latency_metrics, latency_values, color='#2ca02c', alpha=0.8)
plt.axvline(170.320, color='red', linestyle='--', label='Mean (170.320 ms)')
plt.title('MPC Computation Time Profile')
plt.xlabel('Time (ms)')
plt.xlim(0, 500)
plt.legend()
plt.grid(axis='x', linestyle='--', alpha=0.5)

# Add values onto bars
for bar in bars:
    width = bar.get_width()
    plt.text(width + 5, bar.get_y() + bar.get_height()/2, f'{width:.2f} ms', 
             va='center', ha='left', fontsize=9)

plt.tight_layout()
plt.show()
