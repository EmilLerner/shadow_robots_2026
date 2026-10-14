import xsensdeviceapi as xda
import csv
import time
import os

from wirelessMasterCallback import WirelessMasterCallback
from mtwManipulator import MtwManipulator


# ============================================================
# SETTINGS
# ============================================================

update_rate = 80
radio_channel = 19

output_file = "imu_dataset.csv"


# ============================================================
# SENSOR MAPPING
# ============================================================

# Actual IDs from your Xsens system
#
# S2 = wrist
# S3 = forearm
# S4 = biceps / upper arm

mtws = {
    "00B4F199": "S2",
    "00B4F1B3": "S3",
    "00B4F744": "S4"
}


# Final dataset columns
DATASET_COLS = [

    "timestamp",

    # S2
    "S2_ACC_X_R",
    "S2_ACC_Y_R",
    "S2_ACC_Z_R",
    "S2_GYRO_X_R",
    "S2_GYRO_Y_R",
    "S2_GYRO_Z_R",
    "S2_QUAT_W_R",
    "S2_QUAT_X_R",
    "S2_QUAT_Y_R",
    "S2_QUAT_Z_R",

    # S3
    "S3_ACC_X_R",
    "S3_ACC_Y_R",
    "S3_ACC_Z_R",
    "S3_GYRO_X_R",
    "S3_GYRO_Y_R",
    "S3_GYRO_Z_R",
    "S3_QUAT_W_R",
    "S3_QUAT_X_R",
    "S3_QUAT_Y_R",
    "S3_QUAT_Z_R",

    # S4
    "S4_ACC_X_R",
    "S4_ACC_Y_R",
    "S4_ACC_Z_R",
    "S4_GYRO_X_R",
    "S4_GYRO_Y_R",
    "S4_GYRO_Z_R",
    "S4_QUAT_W_R",
    "S4_QUAT_X_R",
    "S4_QUAT_Y_R",
    "S4_QUAT_Z_R"
]

# ============================================================
# CONNECT TO XSENS
# ============================================================

print("\nConnecting to Xsens...")

control = xda.XsControl.construct()

wireless_master_callback = WirelessMasterCallback()


mtw_manipulator = MtwManipulator(
    control,
    wireless_master_callback,
    update_rate,
    radio_channel,
    mtws
)


# ============================================================
# SCAN FOR DEVICES
# ============================================================

print("\nScanning for Xsens devices...")

mtw_manipulator.scanXsDevices()


# ============================================================
# CONFIGURATION MODE
# ============================================================

print("\nStarting configuration mode...")

mtw_manipulator.startConfigMode()


# ============================================================
# WAIT FOR MTWs
# ============================================================

print("\nWaiting for sensor connections...")

mtw_manipulator.waitForConnections()


# ============================================================
# START MEASUREMENT
# ============================================================

print("\nStarting measurement...")

mtw_manipulator.startMeasurement()


# ============================================================
# RESET ORIENTATION
# ============================================================

print("\nResetting orientation data...")

mtw_manipulator.resetOrientationData()


# ============================================================
# START RECORDING
# ============================================================

print("\nStarting recording...")

mtw_manipulator.startRecording()


# ============================================================
# OPEN DATASET
# ============================================================

# Make sure an old CSV is not open in Excel
if os.path.exists(output_file):

    print(
        f"\nWARNING: {output_file} already exists."
    )

    print(
        "The new recording will overwrite it."
    )


try:

    csv_file = open(
        output_file,
        "w",
        newline=""
    )

except PermissionError:

    print(
        f"\nERROR: Cannot open {output_file}."
    )

    print(
        "Make sure the CSV is not open in Excel or another program."
    )

    raise


writer = csv.writer(csv_file)

writer.writerow(DATASET_COLS)


# ============================================================
# START DATA COLLECTION
# ============================================================

print("\n========================================")
print("        IMU DATA COLLECTION")
print("========================================")

print("S2 = Wrist")
print("S3 = Forearm")
print("S4 = Biceps / Upper Arm")

print("\nCollecting:")

print("  - Accelerometer")
print("  - Gyroscope")
print("  - Quaternion")

print("\nMove your arm naturally.")

print("Press Ctrl+C to stop recording.\n")


start_time = time.perf_counter()

sample_count = 0


try:

    while True:

        # ====================================================
        # GET CALLBACKS
        # ====================================================

        callbacks = mtw_manipulator.getMtwCallbacks()


        # ====================================================
        # TEMPORARY SENSOR DATA
        # ====================================================

        sensor_data = {

            "S2": None,
            "S3": None,
            "S4": None
        }


        # ====================================================
        # READ ONE PACKET FROM EACH SENSOR
        # ====================================================

        for callback in callbacks:

            if not callback.dataAvailable():
                continue


            # ------------------------------------------------
            # Get packet
            # ------------------------------------------------

            packet = callback.getOldestPacket()


            # ------------------------------------------------
            # Get sensor ID
            # ------------------------------------------------

            sensor_id = str(
                callback.device().deviceId()
            )


            # ------------------------------------------------
            # Convert sensor ID to body part
            # ------------------------------------------------

            bodypart = mtws.get(sensor_id)


            if bodypart is None:

                callback.deleteOldestPacket()

                continue


            # =================================================
            # ACCELEROMETER
            # =================================================

            acceleration = packet.calibratedAcceleration()

            acc_x = float(acceleration[0])
            acc_y = float(acceleration[1])
            acc_z = float(acceleration[2])


            # =================================================
            # GYROSCOPE
            # =================================================

            gyroscope = packet.calibratedGyroscopeData()

            gyro_x = float(gyroscope[0])
            gyro_y = float(gyroscope[1])
            gyro_z = float(gyroscope[2])


            # =================================================
            # QUATERNION
            # =================================================

            quaternion = packet.orientationQuaternion()

            quat_w = float(quaternion[0])
            quat_x = float(quaternion[1])
            quat_y = float(quaternion[2])
            quat_z = float(quaternion[3])


            # =================================================
            # STORE DATA
            # =================================================

            sensor_data[bodypart] = [

                acc_x,
                acc_y,
                acc_z,

                gyro_x,
                gyro_y,
                gyro_z,

                quat_w,
                quat_x,
                quat_y,
                quat_z
            ]


            # ------------------------------------------------
            # Remove packet from callback queue
            # ------------------------------------------------

            callback.deleteOldestPacket()


        # ====================================================
        # ONLY SAVE WHEN ALL THREE SENSORS HAVE DATA
        # ====================================================

        if (

            sensor_data["S2"] is not None
            and
            sensor_data["S3"] is not None
            and
            sensor_data["S4"] is not None

        ):

            timestamp = (
                time.perf_counter() - start_time
            )


            # =================================================
            # CREATE CSV ROW
            # =================================================

            row = [

                timestamp,


                # -------------------------
                # S2
                # -------------------------

                sensor_data["S2"][0],
                sensor_data["S2"][1],
                sensor_data["S2"][2],

                sensor_data["S2"][3],
                sensor_data["S2"][4],
                sensor_data["S2"][5],

                sensor_data["S2"][6],
                sensor_data["S2"][7],
                sensor_data["S2"][8],
                sensor_data["S2"][9],


                # -------------------------
                # S3
                # -------------------------

                sensor_data["S3"][0],
                sensor_data["S3"][1],
                sensor_data["S3"][2],

                sensor_data["S3"][3],
                sensor_data["S3"][4],
                sensor_data["S3"][5],

                sensor_data["S3"][6],
                sensor_data["S3"][7],
                sensor_data["S3"][8],
                sensor_data["S3"][9],


                # -------------------------
                # S4
                # -------------------------

                sensor_data["S4"][0],
                sensor_data["S4"][1],
                sensor_data["S4"][2],

                sensor_data["S4"][3],
                sensor_data["S4"][4],
                sensor_data["S4"][5],

                sensor_data["S4"][6],
                sensor_data["S4"][7],
                sensor_data["S4"][8],
                sensor_data["S4"][9]
            ]


            # =================================================
            # WRITE DATA
            # =================================================

            writer.writerow(row)

            csv_file.flush()

            sample_count += 1


            # =================================================
            # DISPLAY PROGRESS
            # =================================================

            if sample_count % update_rate == 0:

                elapsed = (
                    time.perf_counter() - start_time
                )

                actual_rate = (
                    sample_count / elapsed
                )

                print(
                    f"Samples: {sample_count:6d} | "
                    f"Time: {elapsed:6.1f} s | "
                    f"Rate: {actual_rate:6.1f} Hz"
                )


        # Small delay
        time.sleep(0.001)


# ============================================================
# STOP RECORDING
# ============================================================

except KeyboardInterrupt:

    print("\n\nStopping recording...")


finally:

    csv_file.close()


    try:

        mtw_manipulator.stopRecording()

    except Exception:

        pass


    try:

        mtw_manipulator.stopMeasurement()

    except Exception:

        pass


    print("\n========================================")
    print("           RECORDING COMPLETE")
    print("========================================")

    print(
        f"Samples recorded: {sample_count}"
    )

    print(
        f"Dataset saved to: {output_file}"
    )

    if sample_count > 0:

        elapsed = (
            time.perf_counter() - start_time
        )

        print(
            f"Average sample rate: "
            f"{sample_count / elapsed:.2f} Hz"
        )