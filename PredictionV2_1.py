import h5py
import numpy as np
import pandas as pd
import math

# Load HDF5 trial
def load_h5_trial(filename, dataset_path):

    print("\nLoading HDF5 dataset:")
    print(dataset_path)

    with h5py.File(filename, "r") as h5_file:
        dataset = h5_file[dataset_path]
        raw_data = dataset[:]
        column_names = dataset.attrs["column_names"]
        columns = []
        for name in column_names:
            if isinstance(name, bytes):
                name = name.decode("utf-8")
            columns.append(str(name))
            
    data = pd.DataFrame(raw_data, columns=columns)

    # Convert all values to numeric
    for column in data.columns:
        data[column] = pd.to_numeric(data[column], errors="coerce")

    return data


# Predict human motion
def predict_human_motion(model, imu_data, imu_features):

    # Convert IMU values into model input
    X = pd.DataFrame([imu_data], columns=imu_features)

    # Predict future joint angles
    predicted_angles = model.predict(X)

    return predicted_angles[0]


# Human forward kinematics
def human_forward_kinematics(joint_angles, upper_arm_length, forearm_length, hand_length):
    (
        shoulder_flex,
        shoulder_add,
        shoulder_rot,
        elbow_flex,
        pro_sup,
        wrist_flex,
        wrist_dev
    ) = joint_angles

    # Convert degrees to radians
    shoulder_flex = math.radians(shoulder_flex)
    shoulder_add = math.radians(shoulder_add)
    shoulder_rot = math.radians(shoulder_rot)
    elbow_flex = math.radians(elbow_flex)
    pro_sup = math.radians(pro_sup)
    wrist_flex = math.radians(wrist_flex)
    wrist_dev = math.radians(wrist_dev)


    # Shoulder rotation

    # Rotation around Z
    Rz = np.array([
        [math.cos(shoulder_add), -math.sin(shoulder_add), 0],
        [math.sin(shoulder_add),  math.cos(shoulder_add), 0],
        [0, 0, 1]
    ])

    # Rotation around Y
    Ry = np.array([
        [ math.cos(shoulder_flex), 0, math.sin(shoulder_flex)],
        [0, 1, 0],
        [-math.sin(shoulder_flex), 0, math.cos(shoulder_flex)]
    ])

    # Rotation around X
    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(shoulder_rot), -math.sin(shoulder_rot)],
        [0, math.sin(shoulder_rot),  math.cos(shoulder_rot)]
    ])

    # Combine shoulder rotations
    R_shoulder = Rz @ Ry @ Rx


    # Upper arm
    upper_arm_vector = np.array([upper_arm_length, 0, 0])
    elbow_position = (R_shoulder @ upper_arm_vector)


    # Elbow rotation

    R_elbow = np.array([
        [math.cos(elbow_flex), -math.sin(elbow_flex), 0],
        [math.sin(elbow_flex),  math.cos(elbow_flex), 0],
        [0, 0, 1]
    ])

    R_forearm = (R_shoulder @ R_elbow)


    # Pronation/supination

    R_pro_sup = np.array([
        [1, 0, 0],
        [0, math.cos(pro_sup), -math.sin(pro_sup)],
        [0, math.sin(pro_sup),  math.cos(pro_sup)]
    ])

    R_forearm_with_pro_sup = (R_forearm @ R_pro_sup)


    # Forearm
    forearm_vector = np.array([forearm_length, 0, 0])
    wrist_position = (elbow_position + R_forearm_with_pro_sup @ forearm_vector)


    # Wrist

    R_wrist_flex = np.array([
        [ math.cos(wrist_flex), 0, math.sin(wrist_flex)],
        [0, 1, 0],
        [-math.sin(wrist_flex), 0, math.cos(wrist_flex)]
    ])

    R_wrist_dev = np.array([
        [math.cos(wrist_dev), -math.sin(wrist_dev), 0],
        [math.sin(wrist_dev),  math.cos(wrist_dev), 0],
        [0, 0, 1]
    ])


    # Complete hand orientation
    R_hand = (R_forearm_with_pro_sup @ R_wrist_flex @ R_wrist_dev)

    # Hand/end-effector position
    hand_vector = np.array([hand_length, 0, 0])
    hand_position = (wrist_position + R_hand @ hand_vector)
    return hand_position, R_hand


# Calculate prediction error
def calculate_prediction_error(predicted_angles, actual_angles, upper_arm_length, forearm_length, hand_length):

    predicted_position, predicted_rotation = (
        human_forward_kinematics(predicted_angles, upper_arm_length, forearm_length, hand_length)
    )

    actual_position, actual_rotation = (
        human_forward_kinematics(actual_angles, upper_arm_length, forearm_length, hand_length)
    )

    # Position error
    position_error = np.linalg.norm(predicted_position - actual_position)

    # Rotation error
    rotation_error_matrix = (predicted_rotation.T @ actual_rotation)
    rotation_error = np.arccos(np.clip((np.trace(rotation_error_matrix) - 1)/2, -1.0, 1.0))
    rotation_error_degrees = np.rad2deg(rotation_error)

    return (
        predicted_position,
        predicted_rotation,
        actual_position,
        actual_rotation,
        position_error,
        rotation_error_degrees
    )


# Get prediction
def get_prediction(model, data, current_index, sample_shift, imu_features, target_angles, upper_arm_length, forearm_length, hand_length):

    # Current imu data
    current_imu = data.loc[current_index, imu_features].values

    # Predict future human joingangles
    predicted_human_angles = (predict_human_motion(model, current_imu, imu_features))

    # Actual future human angles
    future_index = (current_index + sample_shift)
    actual_human_angles = data.loc[future_index, target_angles].values

    # Calculate position/rotation/error
    (
        predicted_position,
        predicted_rotation,
        actual_position,
        actual_rotation,
        position_error,
        rotation_error_degrees
    ) = calculate_prediction_error(
        predicted_human_angles,
        actual_human_angles,
        upper_arm_length,
        forearm_length,
        hand_length
    )

    return (
        predicted_human_angles,
        actual_human_angles,
        predicted_position,
        predicted_rotation,
        actual_position,
        actual_rotation,
        position_error,
        rotation_error_degrees
    )

