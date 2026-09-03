import numpy as np
import joblib
import VariableConfig
from inverse_kinematics_solver4 import choose_best_ik
from PredictionV2_1 import (load_h5_trial, get_prediction)



# Load training parameters
prediction_horizon = VariableConfig.PREDICTION_HORIZON
imu_features = VariableConfig.IMU_FEATURES
target_features = VariableConfig.TARGET_FEATURES
h5_filename = VariableConfig.H5_FILENAME
sample_time = VariableConfig.SAMPLE_TIME
sample_shift = VariableConfig.SAMPLE_SHIFT
model_filename = VariableConfig.MODEL_FILENAME
dataset_path = VariableConfig.DATASET_PATH

# Load human arm parameters
upper_arm_length = VariableConfig.UPPER_ARM_LENGTH
forearm_length = VariableConfig.FOREARM_LENGTH 
hand_length = VariableConfig.HAND_LENGTH 

# Load IK parameters
robot_a = VariableConfig.ROBOT_A
robot_d = VariableConfig.ROBOT_D
robot_b = VariableConfig.ROBOT_B
robot_tp = VariableConfig.ROBOT_TP

# Start 
print("=" * 70)
print("HUMAN MOTION PREDICTION + UR5 CONTROL")
print("=" * 70)
print(f"\nPrediction horizon: "f"{prediction_horizon * 1000:.0f} ms")

# Load Random Forest model
print("\nLoading Random Forest model...")
model = joblib.load(model_filename)
print("Model loaded successfully.")

print("\nMODEL DEBUG")
print("Model filename:", model_filename)
print("Model type:", type(model))
print("Model contents:", model)

# Load HDF5 data
data = load_h5_trial(h5_filename, dataset_path)
print(f"\nLoaded {len(data)} samples.")


# Check required data columns

required_columns = (imu_features + target_features)
missing_columns = [column for column in required_columns if column not in data.columns]

if missing_columns:
    print("\nERROR: Missing columns:")
    for column in missing_columns:
        print(" -", column)
    raise ValueError("Required columns are missing.")
    
print("All required columns found.")


# Remove invalid data 
data = data.dropna(subset=required_columns).reset_index(drop=True)
print(f"Valid samples: {len(data)}")


# Initial robot configuration
previous_robot_angles = np.deg2rad([0, -90, 0, -90, 0, 0])

# Start prediction
print("\n")
print("=" * 70)
print("STARTING PREDICTION + ROBOT CONTROL")
print("=" * 70)

# Prediction loop
for current_index in range(0, len(data) - sample_shift):

    # get prediction
    (
        predicted_human_angles,
        actual_human_angles,
        predicted_human_position,
        predicted_human_rotation,
        actual_human_position,
        actual_human_rotation,
        position_error,
        rotation_error_degrees
    ) = get_prediction(
        model,
        data,
        current_index,
        sample_shift,
        imu_features,
        target_features,
        upper_arm_length,
        forearm_length,
        hand_length
    )


    # Inverse kinematics

    try:
        robot_target = choose_best_ik(
            previous_robot_angles,
            predicted_human_position,
            predicted_human_rotation,
            robot_a,
            robot_d,
            robot_b,
            robot_tp
        )

    except Exception as error:
        print("\nIK ERROR:")
        print(error)
        continue

    # Send target to MPC/PID controller
    arm_motion.move_with_prediction(rtde_r, rtde_c, robot_target)

    # Update previous robot configuration
    previous_robot_angles = np.array(robot_target)


    # Display results
    print("\n")
    print("-" * 70)
    print(f"Sample: {current_index}")
    print(f"Prediction horizon: "f"{prediction_horizon * 1000:.0f} ms")

    # Predicted human joing angles
    print("\nPredicted human joint angles:")
    print(f"  Shoulder flexion: "f"{predicted_human_angles[0]:.2f}°")
    print(f"  Shoulder adduction: "f"{predicted_human_angles[1]:.2f}°")
    print(f"  Shoulder rotation: "f"{predicted_human_angles[2]:.2f}°")
    print(f"  Elbow flexion: "f"{predicted_human_angles[3]:.2f}°")
    print(f"  Pronation/Supination: "f"{predicted_human_angles[4]:.2f}°")
    print(f"  Wrist flexion: "f"{predicted_human_angles[5]:.2f}°")
    print(f"  Wrist deviation: "f"{predicted_human_angles[6]:.2f}°")

    # Predicted position
    print("\nPredicted human hand position:")
    print(f"  X = "f"{predicted_human_position[0]:.4f} m")
    print(f"  Y = "f"{predicted_human_position[1]:.4f} m")
    print(f"  Z = "f"{predicted_human_position[2]:.4f} m")

    # Actual position
    print("\nActual future human hand position:")
    print(f"  X = "f"{actual_human_position[0]:.4f} m")
    print(f"  Y = "f"{actual_human_position[1]:.4f} m")
    print(f"  Z = "f"{actual_human_position[2]:.4f} m")

    # Position error
    print("\n3D position prediction error:")
    print(f"  {position_error:.4f} m")

    # Rotation error
    print("\nHand rotation prediction error:")
    print(f"  {rotation_error_degrees:.2f}°")

    # Robot target
    print("\nRobot joint targets:")
    print(robot_target)
    print("\nRobot joint targets in degrees:")
    print(np.rad2deg(robot_target))


# Completion
print("\n")
print("=" * 70)
print("SIMULATION COMPLETE")
print("=" * 70)


from wirelessMasterCallback import WirelessMasterCallback
from mtwManipulator import MtwManipulator
from armMotionWithMPC import ArmMotion
import sys
import time
import numpy as np
import xsensdeviceapi as xda
import rtde_control, rtde_receive

def stopWork(rtde_r, rtde_c):
    rtde_c.stopScript()
    rtde_c.speedStop()
    rtde_c.servoStop()
    rtde_c.disconnect()
    rtde_r.disconnect()

if __name__ == '__main__':

    ### CONFIGURABLE FIELDS ###

    ROBOT_IP_RIGHT = "127.0.0.1"
    # ROBOT_IP_RIGHT = "192.168.0.183" # REAL IP, DON'T USE FOR TESTING AND DEBUGGING
    # ROBOT_IP_RIGHT = "172.23.252.37"

    desired_update_rate = 80
    desired_radio_channel = 19

    mtws = {
        "00B4F130": "SHOU R",
        "00B4F1B4": "fARM R",
        "00B4F198": "HAND R"
    }

    ### CONFIGURABLE FIELDS ###

    right_arm = ArmMotion() # Configurable second parameter

    rtde_c_R = rtde_control.RTDEControlInterface(ROBOT_IP_RIGHT)
    rtde_r_R = rtde_receive.RTDEReceiveInterface(ROBOT_IP_RIGHT)

    right_arm.move_to_base(rtde_r_R, rtde_c_R)

    wireless_master_callback = WirelessMasterCallback()

    print("Constructing XsControl...")
    control = xda.XsControl.construct()
    if control is None:
        print("Failed to construct XsControl instance.")
        sys.exit(1)

    manipulator = MtwManipulator(control, wireless_master_callback, desired_update_rate, desired_radio_channel, mtws)
    mtw_callbacks = []

    try:
        manipulator.scanXsDevices()
        manipulator.startConfigMode()
        manipulator.waitForConnections()
        manipulator.startMeasurement()
        manipulator.resetOrientationData()
        manipulator.startRecording()

        mtw_callbacks = manipulator.getMtwCallbacks()

        quat_data = {idx: xda.XsQuaternion.identity() for idx in mtws.values()}

        # This function checks for user input to break the loop
        def user_input_ready():
            return False  # Replace this with your method to detect user input

        while not user_input_ready():
            time.sleep(0)

            new_data_available = False
            for i in range(len(mtw_callbacks)):
                if mtw_callbacks[i].dataAvailable():
                    new_data_available = True
                    packet = mtw_callbacks[i].getOldestPacket()
                    mtw_deviceID = str(mtw_callbacks[i].device().deviceId())
                    bodypart_name = mtws.get(mtw_deviceID)
                    quat_data[bodypart_name] = packet.orientationQuaternion()
                    mtw_callbacks[i].deleteOldestPacket()

            if new_data_available:
                side_quat_R = {k[:-2].strip(): v for k, v in quat_data.items() if k.endswith("R")}
                right_arm.move_with_sensors(rtde_r_R, rtde_c_R, side_quat_R)

    except Exception as ex:
        print(ex)
        print("****ABORT****")
        stopWork(rtde_r_R, rtde_c_R)
    except:
        print("An unknown fatal error has occurred. Aborting.")
        print("****ABORT****")
        stopWork(rtde_r_R, rtde_c_R)

    print("Closing XsControl...")
    control.close()

    print("Deleting mtw callbacks...")

    print("Successful exit.")
    print("Press [ENTER] to continue.")
    input()
    stopWork(rtde_r_R, rtde_c_R)