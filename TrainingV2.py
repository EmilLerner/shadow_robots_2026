import pandas as pd
import numpy as np
import VariableConfig


# Load dataset needed for random forest model
csv_filename = VariableConfig.CSV_FILENAME

# Load Random Forest parameters
imu_features = VariableConfig.IMU_FEATURES
target_features = VariableConfig.TARGET_FEATURES

def load_csv_dataset(csv_filename):
    print("\nLoading CSV dataset...")

    # Load the CSV
    data = pd.read_csv(csv_filename)

    # Select only the required RF input and output columns
    required_columns = imu_features + target_features

    # Check that all required columns exist
    missing_columns = [
        column for column in required_columns
        if column not in data.columns
    ]

    if missing_columns:
        raise ValueError(
            f"Missing required columns in CSV: {missing_columns}"
        )

    # Keep only the required columns
    data = data[required_columns].copy()

    # Convert values to numeric; invalid values become NaN
    data = data.apply(pd.to_numeric, errors="coerce")

    # Remove rows with missing input or target values
    data = data.dropna(subset=required_columns)

    data = data.reset_index(drop=True)

    print(f"Rows loaded: {len(data)}")
    print(f"Number of IMU input features: {len(imu_features)}")
    print(f"Number of position target features: {len(target_features)}")

    return data
