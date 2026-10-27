import numpy as np
import pandas as pd
import joblib
import matplotlib.pyplot as plt
import DualVariableConfig
from DualTrainingV2 import load_csv_dataset
from sklearn.metrics import mean_squared_error



# ============================================================
# LOAD CONFIGURATION
# ============================================================

imu_features = DualVariableConfig.IMU_FEATURES
target_features = DualVariableConfig.TARGET_FEATURES
model_filename = DualVariableConfig.COMBINED_MODEL
sample_shift = DualVariableConfig.SAMPLE_SHIFT

# Make sure TEST_FILENAME is defined in DualVariableConfig.py
test_filename = DualVariableConfig.PROCESSED_FILENAME


# ============================================================
# LOAD MODEL AND TEST DATASET
# ============================================================

print("\nLoading trained Random Forest model...")
model = joblib.load(model_filename)

print("Loading separate test dataset...")
dataset = load_csv_dataset(test_filename)

print(f"Dataset samples: {len(dataset)}")
print(f"Input features: {len(imu_features)}")
print(f"Target coordinates: {len(target_features)}")

if len(target_features) != 18:
    raise ValueError(
        f"Expected 18 target coordinates, got {len(target_features)}."
    )

missing_imu = [col for col in imu_features if col not in dataset.columns]
missing_targets = [
    col for col in target_features if col not in dataset.columns
]

if missing_imu or missing_targets:
    raise ValueError(
        f"Missing IMU columns: {missing_imu}\n"
        f"Missing target columns: {missing_targets}"
    )


# ============================================================
# PREPARE TEST INPUTS AND FUTURE TARGETS
# ============================================================

X = dataset[imu_features].copy()
y = dataset[target_features].copy()

# Match each IMU sample with the target sample SAMPLE_SHIFT
# rows into the future.
y = y.shift(-sample_shift)

valid_rows = X.notna().all(axis=1) & y.notna().all(axis=1)

X_test = X.loc[valid_rows].reset_index(drop=True)
y_test = y.loc[valid_rows].reset_index(drop=True)

if X_test.empty:
    raise ValueError(
        "No valid test samples remain. Check the dataset and SAMPLE_SHIFT."
    )

print(f"Valid test samples: {len(X_test)}")
print(f"Prediction shift: {sample_shift} samples")


# ============================================================
# PREDICT POSITIONS
# ============================================================

print("\nRunning Random Forest predictions...")
predictions = model.predict(X_test)

if predictions.shape[1] != 18:
    raise ValueError(
        f"Expected 18 model outputs, got {predictions.shape[1]}."
    )


# ============================================================
# OVERALL COORDINATE RMSE
# ============================================================

# Like TrainingMain, convert metres to centimetres by multiplying
# the RMSE by 100. This assumes target coordinates are in metres.
overall_mse = mean_squared_error(y_test, predictions)
overall_rmse_cm = np.sqrt(overall_mse) * 100

print("\n" + "=" * 65)
print("OVERALL TEST RESULTS")
print("=" * 65)
print(f"Overall coordinate RMSE: {overall_rmse_cm:.4g} cm")


# ============================================================
# RMSE FOR EACH OF THE 18 COORDINATES
# ============================================================

print("\n" + "=" * 65)
print("INDIVIDUAL COORDINATE RMSE")
print("=" * 65)

for i, target in enumerate(target_features):
    target_mse = mean_squared_error(
        y_test.iloc[:, i],
        predictions[:, i]
    )
    target_rmse = np.sqrt(target_mse) * 100

    print(f"{target:<25} {target_rmse:.4g} cm")


# ============================================================
# 3D POSITION ERRORS FOR EACH BODY PART
# ============================================================

# Expected target order:
# 0-2:   Right elbow XYZ
# 3-5:   Right wrist XYZ
# 6-8:   Right hand XYZ
# 9-11:  Left elbow XYZ
# 12-14: Left wrist XYZ
# 15-17: Left hand XYZ

body_parts = {
    "Right elbow": (0, 3),
    "Right wrist": (3, 6),
    "Right hand": (6, 9),
    "Left elbow": (9, 12),
    "Left wrist": (12, 15),
    "Left hand": (15, 18),
}

actual = y_test.to_numpy()
predicted = np.asarray(predictions)

errors_by_part_cm = {}
mean_errors_cm = {}
rmse_errors_cm = {}

print("\n" + "=" * 65)
print("3D POSITION ERRORS")
print("=" * 65)
print(f"{'Body part':<18}{'Mean error (cm)':>20}{'3D RMSE (cm)':>20}")

for body_part, (start, end) in body_parts.items():
    actual_xyz = actual[:, start:end]
    predicted_xyz = predicted[:, start:end]

    # Euclidean distance between predicted and actual XYZ positions.
    errors_cm = np.linalg.norm(
        predicted_xyz - actual_xyz,
        axis=1
    )*100

    errors_by_part_cm[body_part] = errors_cm
    mean_errors_cm[body_part] = np.mean(errors_cm)
    rmse_errors_cm[body_part] = np.sqrt(np.mean(errors_cm ** 2))

    print(
        f"{body_part:<18}"
        f"{mean_errors_cm[body_part]:>20.4g}"
        f"{rmse_errors_cm[body_part]:>20.4g}"
    )

# ============================================================
# GRAPH 1: 3D ERROR WORM PLOT
# ============================================================

plt.figure(figsize=(12, 6))

window_size = 25  # Smooth over 25 test samples

for body_part, errors in errors_by_part_cm.items():

    # Smooth the error curve using a rolling average
    smoothed_errors = (
        pd.Series(errors)
        .rolling(
            window=window_size,
            center=True,
            min_periods=1
        )
        .mean()
    )

    plt.plot(
        smoothed_errors,
        label=body_part,
        linewidth=1.8
    )

plt.xlabel("Test sample")
plt.ylabel("3D position error (cm)")
plt.title("3D Position Error Worm Plot Between Predicted and Actual Bodypart Positions (cm)")
plt.legend()
plt.grid(alpha=0.3)
plt.tight_layout()


# ============================================================
# GRAPH 2: MEAN ERROR VS 3D RMSE
# ============================================================

x = np.arange(len(body_parts))
width = 0.35

plt.figure(figsize=(10, 5))

plt.bar(
    x - width / 2,
    list(mean_errors_cm.values()),
    width,
    label="Mean 3D error"
)

plt.bar(
    x + width / 2,
    list(rmse_errors_cm.values()),
    width,
    label="3D RMSE"
)

plt.xticks(x, list(body_parts.keys()), rotation=25)
plt.ylabel("Error (cm)")
plt.title("Mean 3D Error vs 3D RMSE Between Predicted and Actual Human Bodypart Positions (cm)")
plt.legend()
plt.grid(axis="y", alpha=0.3)
plt.tight_layout()

# ============================================================
# GRAPH 3: BOX-AND-WHISKER PLOT OF 3D POSITION ERRORS
# ============================================================

plt.figure(figsize=(10, 6))

body_part_names = list(errors_by_part_cm.keys())
body_part_errors = [
    errors_by_part_cm[name] for name in body_part_names
]

plt.boxplot(
    body_part_errors,
    labels=body_part_names,  # Changed from tick_labels to labels
    showfliers=True
)

plt.ylabel("3D position error (cm)")
plt.xlabel("Body part")
plt.title("Distribution of 3D Position Errors by Bodypart Between Predicted and Actual Bodypart Positions (cm)")
plt.xticks(rotation=25)
plt.grid(axis="y", alpha=0.3)
plt.tight_layout()
plt.show()

# ============================================================
# OVERALL 3D POSITION ERROR
# ============================================================

# Combine errors for all six body parts (already in cm)
all_point_errors_cm = np.concatenate(
    list(errors_by_part_cm.values())
)

overall_mean_3d_cm = np.mean(all_point_errors_cm)
overall_rmse_3d_cm = np.sqrt(
    np.mean(all_point_errors_cm ** 2)
)

print("-" * 65)
print(f"Overall mean 3D error: {overall_mean_3d_cm:.4g} cm")
print(f"Overall 3D RMSE:       {overall_rmse_3d_cm:.4g} cm")
print("=" * 65)

