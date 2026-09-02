import numpy as np
import pandas as pd
import joblib
import VariableConfig
from Training import load_h5_dataset
from sklearn.ensemble import RandomForestRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error

# Load training parameters
prediction_horizon = VariableConfig.PREDICTION_HORIZON
imu_features = VariableConfig.IMU_FEATURES
target_features = VariableConfig.TARGET_FEATURES
h5_filename = VariableConfig.H5_FILENAME
sample_time = VariableConfig.SAMPLE_TIME
sample_shift = VariableConfig.SAMPLE_SHIFT
model_filename = VariableConfig.MODEL_FILENAME
model_info_filename = VariableConfig.MODEL_INFO_FILENAME



# Program information

print("=" * 60)
print("3D RANDOM FOREST JOINT ANGLE TRAINING")
print("=" * 60)
print(f"\nPrediction horizon: "f"{prediction_horizon * 1000:.0f} ms")
print("\nInput features:")

for feature in imu_features: 
    print(" ", feature)

print("\nTarget joint angles:")

for target in target_features: 
    print(" ", target)


# Load all data
datasets = load_h5_dataset(h5_filename)

# Combine datasets
print("\nCombining datasets...")
data = pd.concat(datasets, ignore_index=True)
print(f"Combined rows: {len(data)}")

# Remove missing data
data = data.dropna(subset=(imu_features + target_features))
data = data.reset_index(drop=True)
print(f"Rows after removing missing data: "f"{len(data)}")

# Create input freatures
X = data[imu_features].copy()

# Create future joint angle targets
# Current sensor measurements: X[t]
# Future joint angles: y[t] = joint angles[t + 5]
# Sampling rate = 100 Hz
# 5 samples = 50 ms
y = data[target_features].shift(-sample_shift)

# Remove invalid target rows
valid_rows = y.notna().all(axis=1)
X = X[valid_rows].reset_index(drop=True)
y = y[valid_rows].reset_index(drop=True)
print(f"Training examples created: "f"{len(X)}")

# Train/test split
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2,random_state=42)
print("\nTraining samples:")
print(len(X_train))
print("\nTesting samples:")
print(len(X_test))


# Create Random Forest

print("\nCreating Random Forest...")

model = RandomForestRegressor(
    n_estimators=120, 
    max_depth=30, 
    min_samples_leaf=2, 
    random_state=42, 
    n_jobs=-1
)


# Train Random Forest
print("\nTraining Random Forest...")
model.fit(X_train, y_train)
print("Training complete.")

# Test trained model
print("\nTesting trained model...")
predictions = model.predict(X_test)

# overall RMSE
overall_mse = mean_squared_error(y_test, predictions)
overall_rmse = np.sqrt(overall_mse)


# Individual RMSE

print("\n")
print("=" * 60)
print("MODEL PERFORMANCE")
print("=" * 60)
print(f"\nOverall RMSE: "f"{overall_rmse:.4f} degrees")
print("\nRMSE for individual joint angles:")

for i, target in enumerate(target_features):
    target_mse = mean_squared_error(y_test.iloc[:, i], predictions[:, i])
    target_rmse = np.sqrt(target_mse)
    print(f"{target:<20} "f"{target_rmse:.4f} degrees")


# Save trained model
print("\nSaving trained model...")
joblib.dump(model, model_filename)
print(f"Model saved as: "f"{model_filename}")


# Save model infromation

model_information = {
    "input_features": imu_features,
    "output_features": target_features,
    "sample_shift": sample_shift,
    "sample_time": sample_time,
    "prediction_horizon": prediction_horizon,
    "number_of_inputs": len(imu_features),
    "number_of_outputs": len(target_features)
}

joblib.dump(model_information, model_info_filename)
print(f"Model information saved as: "f"{model_info_filename}")


# Final summary

print("\n")
print("=" * 60)
print("3D TRAINING COMPLETE")
print("=" * 60)
print(f"Input features: "f"{len(imu_features)}")
print(f"Output features: "f"{len(target_features)}")
print("\nOutputs:")

for target in target_features:
    print(f"  {target}")

print(f"\nPrediction horizon: "f"{prediction_horizon * 1000:.0f} ms")
print(f"Training examples: "f"{len(X_train)}")
print(f"Testing examples: "f"{len(X_test)}")
print(f"\nModel file:"f" {model_filename}")
print("=" * 60)