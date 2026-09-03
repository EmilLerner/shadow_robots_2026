# Settings
H5_FILENAME = "All_subjects_data.h5"
MODEL_FILENAME = "human_joint_angle_predictor_3d.pkl"
MODEL_INFO_FILENAME = "human_joint_angle_predictor_info_3d.pkl"
DATASET_PATH = "subject_1/AS/F"
SAMPLE_SHIFT = 5
SAMPLE_TIME = 0.01
PREDICTION_HORIZON = (SAMPLE_SHIFT * SAMPLE_TIME)

# Random Forest input features
IMU_FEATURES = [
    # Wrist - S2
    "ACCX2",
    "ACCY2",
    "ACCZ2",
    "GYROX2",
    "GYROY2",
    "GYROZ2",

    # Forearm - S3
    "ACCX3",
    "ACCY3",
    "ACCZ3",
    "GYROX3",
    "GYROY3",
    "GYROZ3",

    # Biceps - S4
    "ACCX4",
    "ACCY4",
    "ACCZ4",
    "GYROX4",
    "GYROY4",
    "GYROZ4"
]

# Random Forest outputs
TARGET_FEATURES = [
    "arm_flex_r",
    "arm_add_r",
    "arm_rot_r",
    "elbow_flex_r",
    "pro_sup_r",
    "wrist_flex_r",
    "wrist_dev_r"
]


# Human arm dimensions (m)
UPPER_ARM_LENGTH = 0.30
FOREARM_LENGTH = 0.25
HAND_LENGTH = 0.15

# Robot IK parameters
ROBOT_A = [0.0, 0.4251, 0.39215, 0.0, 0.0, 0.0]
ROBOT_D = [0.0, 0.0, 0.0, 0.11000, 0.09475, 0.0]
ROBOT_B = 0.0892
ROBOT_TP = 0.07495