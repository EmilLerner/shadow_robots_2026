import h5py
import pandas as pd
import numpy as np
import joblib
import VariableConfig
from sklearn.ensemble import RandomForestRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error

# Load Random Forest parameters
imu_features = VariableConfig.IMU_FEATURES
target_features = VariableConfig.TARGET_FEATURES


# Load HDF5 dataset

def load_h5_dataset(filename):

    print("\nLoading H5 dataset...")
    all_data = []

    with h5py.File(filename, "r") as h5_file:
        for subject_name in h5_file.keys(): 
            subject = h5_file[subject_name]
            print(f"\nProcessing {subject_name}...")

            for trial_name in subject.keys():
                trial = subject[trial_name]
                if isinstance(trial, h5py.Dataset):
                    process_dataset(trial, subject_name, trial_name, all_data)
                else:
                    for speed_name in trial.keys():
                        dataset = trial[speed_name]
                        if isinstance(dataset, h5py.Dataset):
                            process_dataset(dataset, subject_name, f"{trial_name}/{speed_name}", all_data, imu_features, target_features)

    if len(all_data) == 0:
        raise ValueError("No suitable datasets were found.")

    print(f"\nTotal datasets loaded: "f"{len(all_data)}")
    return all_data


# Process individual dataset
def process_dataset(dataset, subject_name, trial_name, all_data, imu_features, target_features):

    # Get column names

    column_names = dataset.attrs.get("column_names")

    if column_names is None:
        print(f"Skipping {subject_name}/{trial_name}: ""no column_names attribute.")
        return


    # Convert column names

    columns = []

    for column in column_names:
        if isinstance(column, bytes):
            column = column.decode("utf-8")

        columns.append(str(column))


    # Check required columns

    required_columns = (imu_features + target_features)
    missing_columns = [column for column in required_columns if column not in columns]

    if missing_columns:
        print(f"Skipping {subject_name}/{trial_name}: ""required columns missing.")
        return


    # Find only required columns
    column_indices = [columns.index(column) for column in required_columns]

    # Read data
    raw_data = dataset[:]


    # Convert data to numeric

    selected_data = []

    for row in raw_data:
        row_values = []

        for index in column_indices:
            value = row[index]

            # Convert bytes to string
            if isinstance(value, bytes):
                value = value.decode("utf-8").strip()

            # Empty values
            if value == "":
                row_values.append(np.nan)
            else:
                try: row_values.append(float(value))
                except (ValueError, TypeError): row_values.append(np.nan)

        selected_data.append(row_values)


    # Create dataset
    dataframe = pd.DataFrame(selected_data, columns=required_columns)

    # Store dataset
    all_data.append(dataframe)
    print(f"  Loaded {subject_name}/{trial_name}: "f"{len(dataframe)} rows")
