import DualVariableConfig
import numpy as np
import pandas as pd
from armMotionV2 import ArmMotion

# Load your recorded Xsens dataset
raw_filename = DualVariableConfig.RAW_FILENAME

# Set name of file needed for training
processed_filename = DualVariableConfig.PROCESSED_FILENAME
df = pd.read_csv(raw_filename)

target_features = DualVariableConfig.TARGET_FEATURES
quat_cols = DualVariableConfig.QUAT_COLUMNS
L_upper = DualVariableConfig.UPPER_ARM_LENGTH
L_forearm = DualVariableConfig.FOREARM_LENGTH
L_hand = DualVariableConfig.HAND_LENGTH

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

if len(target_features) != 18:
    raise ValueError(
        f"Expected 18 target features, "
        f"but found {len(target_features)}."
    )

# Create an ArmMotion object to reuse its position calculation.
# These arguments are only used to initialize the class here.
arm = ArmMotion(arms_dim, orientation, devices={})

# Keep the original rows and timestamps
valid = df[required].notna().all(axis=1)

# Create target columns, initially empty
for feature in target_features:
    df[feature] = np.nan

def get_quat_data(row):
    return {
        sensor: tuple(row[col] for col in columns)
        for sensor, columns in quat_cols.items()
    }

positions = []

for _, row in df.loc[valid].iterrows():
    quat_data = get_quat_data(row)

     # Right arm: S1 shoulder, S2 forearm, S3 hand
    right_quat_data = {
        "SHOU": quat_data["RSHOU"],
        "fARM": quat_data["RfARM"],
        "HAND": quat_data["RHAND"]
    }

    # Left arm: S4 shoulder, S5 forearm, S6 hand
    left_quat_data = {
        "SHOU": quat_data["LSHOU"],
        "fARM": quat_data["LfARM"],
        "HAND": quat_data["LHAND"]
    }


     # Each call returns 9 position values
    right_positions = arm.calculate_human_positions(
        right_quat_data
    )

    left_positions = arm.calculate_human_positions(
        left_quat_data
    )

    # Combine into 18 values
    combined_positions = np.concatenate([
        right_positions,
        left_positions
    ])

    positions.append(combined_positions)

if positions:
    df.loc[valid, target_features] = np.vstack(positions)

df.to_csv(processed_filename, index=False)

print("Data successfully saved to 'dual_xsens_dataset_with_positions.csv'")