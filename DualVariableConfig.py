import numpy as np

# Settings
RAW_FILENAME = "dual_imu_dataset1.csv"
PROCESSED_FILENAME = "dual_xsens_dataset_with_positions5.csv"
TEST_FILENAME = "dual_xsens_dataset_with_positions_test.csv"
MODEL_FILENAME = "dual_human_joint_coordinates_predictor_3d1.pkl"
COMBINED_MODEL = "combined_RF_model.pkl"
MODEL_INFO_FILENAME = "dual_human_joint_coordinates_predictor_info_3d1.pkl"
DATASET_PATH = "subject_1/AS/F"
SAMPLE_RATE = 80 # Hz
RADIO_CHANNEL = 19
SAMPLE_SHIFT = 5
SAMPLE_TIME = 1.0/SAMPLE_RATE
PREDICTION_HORIZON = (SAMPLE_SHIFT * SAMPLE_TIME)
MAX_TIME_SKEW = 0.5/SAMPLE_RATE # Maximum allowed difference between sensor packet arrival times.

# The intended IMU features are ACC xyz + GYRO xyz for each sensor.
EXPECTED_INPUT_COUNT = 36
EXPECTED_OUTPUT_COUNT = 18

# Random Forest input features
IMU_FEATURES = [
    # right shoulder - S1
    "S1_ACC_X_R",
    "S1_ACC_Y_R",
    "S1_ACC_Z_R",
    "S1_GYRO_X_R",
    "S1_GYRO_Y_R",
    "S1_GYRO_Z_R",

    # right forearm - S2
    "S2_ACC_X_R",
    "S2_ACC_Y_R",
    "S2_ACC_Z_R",
    "S2_GYRO_X_R",
    "S2_GYRO_Y_R",
    "S2_GYRO_Z_R",

    # right hand - S3
    "S3_ACC_X_R",
    "S3_ACC_Y_R",
    "S3_ACC_Z_R",
    "S3_GYRO_X_R",
    "S3_GYRO_Y_R",
    "S3_GYRO_Z_R",

    # left shoulder - S4
    "S4_ACC_X_L",
    "S4_ACC_Y_L",
    "S4_ACC_Z_L",
    "S4_GYRO_X_L",
    "S4_GYRO_Y_L",
    "S4_GYRO_Z_L",

    # left forearm - S5
    "S5_ACC_X_L",
    "S5_ACC_Y_L",
    "S5_ACC_Z_L",
    "S5_GYRO_X_L",
    "S5_GYRO_Y_L",
    "S5_GYRO_Z_L",

    # left hand - S6
    "S6_ACC_X_L",
    "S6_ACC_Y_L",
    "S6_ACC_Z_L",
    "S6_GYRO_X_L",
    "S6_GYRO_Y_L",
    "S6_GYRO_Z_L"
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
    "hand_z_r",

    "elbow_x_l",
    "elbow_y_l",
    "elbow_z_l",
    "wrist_x_l",
    "wrist_y_l",
    "wrist_z_l",
    "hand_x_l",
    "hand_y_l",
    "hand_z_l"
]

QUAT_COLUMNS = {
    "RSHOU": ["S1_QUAT_W_R", "S1_QUAT_X_R","S1_QUAT_Y_R", "S1_QUAT_Z_R"],
    "RfARM": ["S2_QUAT_W_R", "S2_QUAT_X_R", "S2_QUAT_Y_R", "S2_QUAT_Z_R"],
    "RHAND": ["S3_QUAT_W_R", "S3_QUAT_X_R", "S3_QUAT_Y_R", "S3_QUAT_Z_R"],
    
    "LSHOU": ["S4_QUAT_W_L", "S4_QUAT_X_L","S4_QUAT_Y_L", "S4_QUAT_Z_L"],
    "LfARM": ["S5_QUAT_W_L", "S5_QUAT_X_L", "S5_QUAT_Y_L", "S5_QUAT_Z_L"],
    "LHAND": ["S6_QUAT_W_L", "S6_QUAT_X_L", "S6_QUAT_Y_L", "S6_QUAT_Z_L"]
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

