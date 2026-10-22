from pathlib import Path
from collections import deque
import queue

import joblib
import numpy as np
import pandas as pd

import DualVariableConfig
from armMotionV2 import ArmMotion

upper_arm_length = DualVariableConfig.UPPER_ARM_LENGTH
forearm_length = DualVariableConfig.FOREARM_LENGTH
hand_length = DualVariableConfig.HAND_LENGTH

# Use the exact dimensions and orientation configuration from
# your existing ArmMotion setup.
arms_dim = [upper_arm_length, forearm_length, hand_length]

orientation = DualVariableConfig.ORIENTATION  

# Only one instance is needed because calculate_human_positions()
# accepts the quaternion data for either arm.
arm_motion = ArmMotion(
    arms_dim,
    orientation,
    devices={}
)

# Model loading

def load_model(model_filename):
    """Load the trained 18-output Random Forest model."""

    path = path(model_filename)

    if not path.exists():
        raise FileNotFoundError(
            f"Could not find model: {path.resolve()}\n"
            "Check MODEL_FILENAME in VariableConfig.py and ensure "
            "the 18-output Cartesian position model has been trained."
        )

    model = joblib.load(path)

    if not hasattr(model, "predict"):
        raise TypeError("Loaded model does not provide predict().")

    return model


import queue

# This queue must be shared with your Xsens acquisition code.
sample_queue = queue.Queue(maxsize=400)


def read_live_sample(timeout=2.0):
    """
    Return one complete sample from the Xsens acquisition loop.

    The acquisition loop must place dictionaries containing
    all six sensors' features and quaternions into sample_queue.
    """

    try:
        sample = sample_queue.get(timeout=timeout)
    except queue.Empty:
        raise TimeoutError(
            "No complete Xsens sample received within "
            f"{timeout} seconds."
        )

    required_sensors = {
        "S1": "R",
        "S2": "R",
        "S3": "R",
        "S4": "L",
        "S5": "L",
        "S6": "L",
    }

    required_measurements = [
        "ACC_X", "ACC_Y", "ACC_Z",
        "GYRO_X", "GYRO_Y", "GYRO_Z",
        "QUAT_W", "QUAT_X", "QUAT_Y", "QUAT_Z",
    ]

    required_keys = [
        f"{sensor}_{measurement}_{side}"
        for sensor, side in required_sensors.items()
        for measurement in required_measurements
    ]

    missing = [key for key in required_keys if key not in sample]

    if missing:
        raise ValueError(
            f"Incomplete Xsens sample. Missing keys: {missing}"
        )

    return sample

# --------------------------- Actual Cartesian positions ---------------------------

def calculate_actual_positions(sample):
    """
    Calculate the actual elbow, wrist, and hand positions for both arms.

    Return either:
      - a dictionary keyed by TARGET_FEATURES, or
      - a NumPy array in exactly TARGET_FEATURES order.

    The six points must produce 18 Cartesian coordinates:
        Right: elbow, wrist, hand (x, y, z)
        Left:  elbow, wrist, hand (x, y, z)
    """

    # Replace these references with your actual configuration.
    upper_arm_length = DualVariableConfig.UPPER_ARM_LENGTH
    forearm_length = DualVariableConfig.FOREARM_LENGTH
    hand_length = DualVariableConfig.HAND_LENGTH

    # Right arm: S1 shoulder, S2 forearm, S3 hand
    right_quat_data = {
        "SHOU": [
            sample["S1_QUAT_W_R"],
            sample["S1_QUAT_X_R"],
            sample["S1_QUAT_Y_R"],
            sample["S1_QUAT_Z_R"],
        ],
        "fARM": [
            sample["S2_QUAT_W_R"],
            sample["S2_QUAT_X_R"],
            sample["S2_QUAT_Y_R"],
            sample["S2_QUAT_Z_R"],
        ],
        "HAND": [
            sample["S3_QUAT_W_R"],
            sample["S3_QUAT_X_R"],
            sample["S3_QUAT_Y_R"],
            sample["S3_QUAT_Z_R"],
        ],
    }

    # Left arm: S4 shoulder, S5 forearm, S6 hand
    left_quat_data = {
        "SHOU": [
            sample["S4_QUAT_W_L"],
            sample["S4_QUAT_X_L"],
            sample["S4_QUAT_Y_L"],
            sample["S4_QUAT_Z_L"],
        ],
        "fARM": [
            sample["S5_QUAT_W_L"],
            sample["S5_QUAT_X_L"],
            sample["S5_QUAT_Y_L"],
            sample["S5_QUAT_Z_L"],
        ],
        "HAND": [
            sample["S6_QUAT_W_L"],
            sample["S6_QUAT_X_L"],
            sample["S6_QUAT_Y_L"],
            sample["S6_QUAT_Z_L"],
        ],
    }

    # Reuse your existing forward-kinematics implementation.
    right_positions = arm_motion.calculate_human_positions(
        right_quat_data
    )

    left_positions = arm_motion.calculate_human_positions(
        left_quat_data
    )

    # 9 right-arm values + 9 left-arm values = 18 values.
    actual_positions = np.concatenate([
        right_positions,
        left_positions,
    ])

    if actual_positions.shape != (18,):
        raise ValueError(
            f"Expected 18 actual position values, "
            f"got {actual_positions.shape}"
        )

    if not np.all(np.isfinite(actual_positions)):
        raise ValueError("Actual positions contain invalid values.")

    return actual_positions


# --------------------------- Prediction ---------------------------

def predict_positions(
    model,
    imu_sample,
    imu_features,
    target_features,
):
    """
    Predict future Cartesian positions from one live IMU sample.

    Returns a 1D NumPy array in target_features order.
    """

    missing = [
        name for name in imu_features
        if name not in imu_sample
    ]

    if missing:
        raise ValueError(f"Missing IMU features: {missing}")

    X = pd.DataFrame(
        [[imu_sample[name] for name in imu_features]],
        columns=imu_features,
    ).astype(float)

    prediction = np.asarray(
        model.predict(X),
        dtype=float,
    ).reshape(-1)

    if len(prediction) != len(target_features):
        raise ValueError(
            f"Model returned {len(prediction)} outputs; "
            f"expected {len(target_features)}. "
            "Check that the loaded model was trained on all 18 "
            "Cartesian position targets."
        )

    if not np.all(np.isfinite(prediction)):
        raise ValueError(
            "Prediction contains NaN or infinite values."
        )

    return prediction


# --------------------------- Actual position conversion ---------------------------

def actual_positions_to_array(actual_positions, target_features):
    """
    Convert actual positions to the target feature order.

    Accepts a dictionary or an array.
    """

    if isinstance(actual_positions, dict):
        missing = [
            name for name in target_features
            if name not in actual_positions
        ]

        if missing:
            raise ValueError(
                f"Missing actual position values: {missing}"
            )

        actual = np.asarray(
            [actual_positions[name] for name in target_features],
            dtype=float,
        )

    else:
        actual = np.asarray(
            actual_positions,
            dtype=float,
        ).reshape(-1)

    if len(actual) != len(target_features):
        raise ValueError(
            f"Received {len(actual)} actual values; "
            f"expected {len(target_features)}."
        )

    if not np.all(np.isfinite(actual)):
        raise ValueError(
            "Actual positions contain NaN or infinite values."
        )

    return actual


# --------------------------- Error calculation ---------------------------

def calculate_position_errors(
    predicted,
    actual,
    target_features,
):
    """
    Calculate signed coordinate errors and Euclidean point errors.

    Coordinate error:
        predicted coordinate - actual coordinate

    Point error:
        Euclidean distance between predicted and actual point positions.

    Returns:
        coordinate_errors: 18 signed coordinate errors
        point_errors: errors for six points, in metres
    """

    predicted = np.asarray(
        predicted,
        dtype=float,
    ).reshape(-1)

    actual = np.asarray(
        actual,
        dtype=float,
    ).reshape(-1)

    if len(predicted) != len(target_features):
        raise ValueError(
            "Predicted vector length does not match targets."
        )

    if len(actual) != len(target_features):
        raise ValueError(
            "Actual vector length does not match targets."
        )

    if not np.all(np.isfinite(predicted)):
        raise ValueError("Predicted values contain invalid numbers.")

    if not np.all(np.isfinite(actual)):
        raise ValueError("Actual values contain invalid numbers.")

    coordinate_errors = predicted - actual
    point_errors = {}

    for arm in ("r", "l"):
        for point in ("elbow", "wrist", "hand"):

            names = [
                f"{point}_x_{arm}",
                f"{point}_y_{arm}",
                f"{point}_z_{arm}",
            ]

            missing = [
                name for name in names
                if name not in target_features
            ]

            if missing:
                raise ValueError(
                    f"Missing target coordinates for "
                    f"{point}_{arm}: {missing}"
                )

            indices = [
                target_features.index(name)
                for name in names
            ]

            point_errors[f"{point}_{arm}"] = float(
                np.linalg.norm(coordinate_errors[indices])
            )

    return coordinate_errors, point_errors


# --------------------------- Delayed prediction evaluator ---------------------------

class DelayedPredictionEvaluator:
    """
    Match future predictions to actual positions by sample number.

    Example:
        A prediction made at sample 100 with SAMPLE_SHIFT=5
        is compared with actual positions at sample 105.
    """

    def __init__(self, sample_shift, target_features):

        if sample_shift < 1:
            raise ValueError(
                "sample_shift must be at least 1."
            )

        self.sample_shift = int(sample_shift)
        self.target_features = list(target_features)

        self.queue = deque()

        self.coordinate_errors = {
            name: []
            for name in self.target_features
        }

        self.point_errors = {
            f"{point}_{arm}": []
            for arm in ("r", "l")
            for point in ("elbow", "wrist", "hand")
        }

        self.sample_count = 0
        self.matched_count = 0

    def add_prediction(self, prediction):
        """Queue a prediction generated from the current sample."""

        prediction = np.asarray(
            prediction,
            dtype=float,
        ).reshape(-1)

        if len(prediction) != len(self.target_features):
            raise ValueError(
                "Prediction length does not match targets."
            )

        if not np.all(np.isfinite(prediction)):
            raise ValueError(
                "Prediction contains NaN or infinite values."
            )

        self.queue.append({
            "source_sample": self.sample_count,
            "target_sample": self.sample_count + self.sample_shift,
            "prediction": prediction.copy(),
        })

    def compare_if_due(self, actual_positions):
        """
        Compare a due prediction with the current actual positions.

        Call once per live sample, before add_prediction() and
        increment_sample().

        Returns a result dictionary when a prediction is due,
        or None when no prediction is due.
        """

        actual = actual_positions_to_array(
            actual_positions,
            self.target_features,
        )

        if not self.queue:
            return None

        item = self.queue[0]

        if item["target_sample"] > self.sample_count:
            return None

        self.queue.popleft()

        if item["target_sample"] != self.sample_count:
            return {
                "matched": False,
                "source_sample": item["source_sample"],
                "target_sample": item["target_sample"],
            }

        coordinate_errors, point_errors = calculate_position_errors(
            item["prediction"],
            actual,
            self.target_features,
        )

        for i, name in enumerate(self.target_features):
            self.coordinate_errors[name].append(
                float(coordinate_errors[i])
            )

        for name, error in point_errors.items():
            self.point_errors[name].append(error)

        self.matched_count += 1

        return {
            "matched": True,
            "source_sample": item["source_sample"],
            "target_sample": item["target_sample"],
            "predicted": item["prediction"].copy(),
            "actual": actual.copy(),
            "coordinate_errors": coordinate_errors.copy(),
            "point_errors": point_errors.copy(),
            "mean_point_error": float(
                np.mean(list(point_errors.values()))
            ),
        }

    def increment_sample(self):
        """Advance the sample counter once per live sample."""

        self.sample_count += 1

    def get_rmse(self):
        """
        Return coordinate RMSE and point-distance RMSE.

        Both are calculated over all successfully matched predictions.
        """

        coordinate_rmse = {
            name: float(
                np.sqrt(np.mean(np.square(errors)))
            )
            for name, errors in self.coordinate_errors.items()
            if errors
        }

        point_rmse = {
            name: float(
                np.sqrt(np.mean(np.square(errors)))
            )
            for name, errors in self.point_errors.items()
            if errors
        }

        return coordinate_rmse, point_rmse

    def print_rmse(self):

        coordinate_rmse, point_rmse = self.get_rmse()

        print("\n========== RUNNING RMSE ==========")

        if not coordinate_rmse and not point_rmse:
            print("No matched predictions available yet.")

        for name, value in coordinate_rmse.items():
            print(
                f"{name}: {value:.5f} m "
                "(coordinate RMSE)"
            )

        for name, value in point_rmse.items():
            print(
                f"{name}: {value:.5f} m "
                "(point-distance RMSE)"
            )

        print(f"Matched predictions: {self.matched_count}")
        print("==================================\n")
