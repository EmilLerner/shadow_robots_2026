import VariableConfig
import numpy as np
import pandas as pd
from armMotionV2 import ArmMotion

# Load your recorded Xsens dataset
input_filename = "imu_dataset.csv"

# Set name of file needed for training
csv_filename = VariableConfig.CSV_FILENAME
df = pd.read_csv(input_filename)

target_features = VariableConfig.TARGET_FEATURES
quat_cols = VariableConfig.QUAT_COLUMNS
L_upper = VariableConfig.UPPER_ARM_LENGTH
L_forearm = VariableConfig.FOREARM_LENGTH
L_hand = VariableConfig.HAND_LENGTH

# Must match self.orientation in ArmMotion: "INV" or other
orientation  = "INV"

arms_dim = [L_upper, L_forearm, L_hand]

required = [
    col 
    for sensor_cols in quat_cols.values()
    for col in sensor_cols
]

missing = [col for col in required if col not in df.columns]

if missing:
    raise ValueError(
        f"Missing quaternion columns: {missing}\n"
    )

# Create an ArmMotion object to reuse its position calculation.
# These arguments are only used to initialize the class here.
arm = ArmMotion(arms_dim, orientation, devices={})

# Keep the original rows and timestamps
valid = df[required].notna().all(axis=1)

df[target_features] = np.nan

def get_quat_data(row):
    return {
        sensor: tuple(row[col] for col in columns)
        for sensor, columns in quat_cols.items()
    }

positions = []

for _, row in df.loc[valid].iterrows():
    quat_data = get_quat_data(row)

    position = arm.calculate_human_positions(quat_data)
    positions.append(position)

if positions:
    df.loc[valid, target_features] = np.vstack(positions)

df.to_csv(csv_filename, index=False)

print("Data successfully saved to xsens_dataset_with_positions.csv")