import numpy as np

# Settings
INPUT_FILENAME = "imu_dataset.csv"
CSV_FILENAME = "xsens_dataset_with_positions.csv"
MODEL_FILENAME = "human_joint_angle_predictor_3d.pkl"
MODEL_INFO_FILENAME = "human_joint_angle_predictor_info_3d.pkl"
DATASET_PATH = "subject_1/AS/F"
SAMPLE_SHIFT = 5
SAMPLE_TIME = 0.01
PREDICTION_HORIZON = (SAMPLE_SHIFT * SAMPLE_TIME)

# Random Forest input features
IMU_FEATURES = [
    # Wrist - S2
    "S2_ACC_X_R",
    "S2_ACC_Y_R",
    "S2_ACC_Z_R",
    "S2_GYRO_X_R",
    "S2_GYRO_Y_R",
    "S2_GYRO_Z_R",

    # Forearm - S3
    "S3_ACC_X_R",
    "S3_ACC_Y_R",
    "S3_ACC_Z_R",
    "S3_GYRO_X_R",
    "S3_GYRO_Y_R",
    "S3_GYRO_Z_R",

    # Biceps - S4
    "S4_ACC_X_R",
    "S4_ACC_Y_R",
    "S4_ACC_Z_R",
    "S4_GYRO_X_R",
    "S4_GYRO_Y_R",
    "S4_GYRO_Z_R"
]

# Random Forest outputs
TARGET_FEATURES = [
    "elbow_x_r",
    "elbow_y_r",
    "elbow_z_r",
    "wrist_x_r",
    "wrist_y_r",
    "wrist_z_r",
    "hand_x_r",
    "hand_y_r",
    "hand_z_r"
]

QUAT_COLUMNS = {
    "SHOU": ["S4_QUAT_W_R", "S4_QUAT_X_R","S4_QUAT_Y_R", "S4_QUAT_Z_R"],
    "fARM": ["S3_QUAT_W_R", "S3_QUAT_X_R", "S3_QUAT_Y_R", "S3_QUAT_Z_R"],
    "HAND": ["S2_QUAT_W_R", "S2_QUAT_X_R", "S2_QUAT_Y_R", "S2_QUAT_Z_R"],
}

# Human arm dimensions (m)
UPPER_ARM_LENGTH = 0.30
FOREARM_LENGTH = 0.25
HAND_LENGTH = 0.15

# Robot IK parameters
ROBOT_A = [0.0, 0.4251, 0.39215, 0.0, 0.0, 0.0]
ROBOT_D = [0.0, 0.0, 0.0, 0.11000, 0.09475, 0.0]
ROBOT_B = 0.0892
ROBOT_TP = 0.07495

# Safety clearance
MIN_CLEARANCE = 0.045
PROTECTIVE_STOP_DISTANCE = 0.028 # from ursim

JOINT_MIN = np.deg2rad([-360.0, -360.0, -360.0, -360.0, -360.0, -360.0])
JOINT_MAX = np.deg2rad([360.0, 360.0, 360.0, 360.0, 360.0, 360.0])

MAX_JOINT_STEP = np.deg2rad(5.0)

# Must match self.orientation in ArmMotion: "INV" or other
ORIENTATION = "INV"

