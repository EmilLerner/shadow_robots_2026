import xsensdeviceapi as xda
import csv
import time
import os
from wirelessMasterCallback import WirelessMasterCallback
from mtwManipulator import MtwManipulator

# setting
update_rate = 80
radio_channel = 19
output_file = "dual_imu_dataset.csv"

# Allow up to half a nominal sample interval between sensor
# packet-arrival timestamps when aligning six sensor streams.
max_time_skew = 0.5 / update_rate

# Flush CSV data every N saved rows.
flush_rate = update_rate

# Prevent excessive memory use if one sensor stops delivering data.
max_buffer_size = update_rate * 5

# sensor maping
# S1 = right shoulder 
# S2 = right forearm 
# S3 = right hand
# S4 = left shoulder
# S5 = left forearm 
# S6 = left hand

mtws = {
    "00B4F1B1": "S1",
    "00B4F1AE": "S2",
    "00B4F12E": "S3",
    
    "00B4F12F": "S4",
    "00B4F1B3": "S5",
    "00B4F130": "S6" 
}

# Final dataset columns
dataset_cols = [
    "timestamp",

    # S1
    "S1_ACC_X_R",
    "S1_ACC_Y_R",
    "S1_ACC_Z_R",
    "S1_GYRO_X_R",
    "S1_GYRO_Y_R",
    "S1_GYRO_Z_R",
    "S1_QUAT_W_R",
    "S1_QUAT_X_R",
    "S1_QUAT_Y_R",
    "S1_QUAT_Z_R",

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
    "S4_ACC_X_L",
    "S4_ACC_Y_L",
    "S4_ACC_Z_L",
    "S4_GYRO_X_L",
    "S4_GYRO_Y_L",
    "S4_GYRO_Z_L",
    "S4_QUAT_W_L",
    "S4_QUAT_X_L",
    "S4_QUAT_Y_L",
    "S4_QUAT_Z_L",

    # S5
    "S5_ACC_X_L",
    "S5_ACC_Y_L",
    "S5_ACC_Z_L",
    "S5_GYRO_X_L",
    "S5_GYRO_Y_L",
    "S5_GYRO_Z_L",
    "S5_QUAT_W_L",
    "S5_QUAT_X_L",
    "S5_QUAT_Y_L",
    "S5_QUAT_Z_L",

    # S6
    "S6_ACC_X_L",
    "S6_ACC_Y_L",
    "S6_ACC_Z_L",
    "S6_GYRO_X_L",
    "S6_GYRO_Y_L",
    "S6_GYRO_Z_L",
    "S6_QUAT_W_L",
    "S6_QUAT_X_L",
    "S6_QUAT_Y_L",
    "S6_QUAT_Z_L"
]

# connect to xsens

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

    print(f"\nWARNING: {output_file} already exists.")
    print("The new recording will overwrite it.")

try:
    csv_file = open(output_file, "w", newline="")

except PermissionError:
    print(f"\nERROR: Cannot open {output_file}.")
    print("Make sure the CSV is not open in Excel or another program.")
    raise

writer = csv.writer(csv_file)
writer.writerow(dataset_cols)


# start data collection

print("\n========================================")
print("        IMU DATA COLLECTION")
print("========================================")
print("S1 = right shoulder")
print("S2 = right forearm")
print("S3 = right hand")
print("S4 = left shoulder")
print("S5 = left forearm")
print("S6 = left hand")
print("\nCollecting:")
print("  - Accelerometer")
print("  - Gyroscope")
print("  - Quaternion")
print("\nMove your arm naturally.")
print("Press Ctrl+C to stop recording.\n")


start_time = time.perf_counter()
sample_count = 0

sensor_names = ("S1", "S2", "S3", "S4", "S5", "S6")

packet_counts = {sensor: 0 for sensor in sensor_names}

# Keep the latest data across loop iterations.
sensor_data = {sensor: None for sensor in sensor_names}

# Track which sensors have supplied a new packet since
# the last saved row.
updated_since_save = {sensor: False for sensor in sensor_names}

last_report_time = start_time
last_report_count = 0

try:
    while True:
        callbacks = mtw_manipulator.getMtwCallbacks()

        # Read packets from all available sensor callbacks.
        for callback in callbacks:
            sensor_id = str(callback.device().deviceId())
            bodypart = mtws.get(sensor_id)

            while callback.dataAvailable():
                packet = callback.getOldestPacket()

                try:
                    if bodypart is None:
                        continue

                    acceleration = packet.calibratedAcceleration()
                    gyroscope = packet.calibratedGyroscopeData()
                    quaternion = packet.orientationQuaternion()

                    values = [
                        float(acceleration[0]),
                        float(acceleration[1]),
                        float(acceleration[2]),
                        float(gyroscope[0]),
                        float(gyroscope[1]),
                        float(gyroscope[2]),
                        float(quaternion[0]),
                        float(quaternion[1]),
                        float(quaternion[2]),
                        float(quaternion[3]),
                    ]

                    sensor_data[bodypart] = values
                    updated_since_save[bodypart] = True
                    packet_counts[bodypart] += 1

                finally:
                    callback.deleteOldestPacket()

        # Save only after every sensor has supplied new data.
        all_ready = all(
            sensor_data[sensor] is not None
            for sensor in sensor_names
        )

        all_updated = all(
            updated_since_save[sensor]
            for sensor in sensor_names
        )

        if all_ready and all_updated:
            timestamp = time.perf_counter() - start_time

            row = [timestamp]

            # Preserve the existing CSV column order.
            for sensor in sensor_names:
                row.extend(sensor_data[sensor])

            writer.writerow(row)
            sample_count += 1

            # Each sensor must contribute another packet
            # before the next row can be saved.
            updated_since_save = {
                sensor: False for sensor in sensor_names
            }

            # Flush once per nominal second of saved rows.
            if sample_count % flush_rate == 0:
                csv_file.flush()

        # Report progress every two seconds, even if no
        # complete sample has been saved.
        now = time.perf_counter()

        if now - last_report_time >= 2.0:
            elapsed = now - start_time
            interval = now - last_report_time

            new_samples = sample_count - last_report_count

            recent_rate = (
                new_samples / interval if interval > 0 else 0.0
            )

            average_rate = (
                sample_count / elapsed if elapsed > 0 else 0.0
            )

            print(
                f"\rSamples: {sample_count:7d} | "
                f"Time: {elapsed:7.1f} s | "
                f"Average: {average_rate:6.2f} Hz | "
                f"Recent: {recent_rate:6.2f} Hz",
                flush=True
            )

            print(
                "Packets: "
                + " | ".join(
                    f"{sensor}: {packet_counts[sensor]}"
                    for sensor in sensor_names
                ),
                flush=True
            )

            last_report_time = now
            last_report_count = sample_count

        # Avoid a busy loop when no packets are available.
        time.sleep(0.001)

except KeyboardInterrupt:
    print("\n\nStopping recording...")

finally:
    csv_file.flush()
    csv_file.close()

    try:
        mtw_manipulator.stopRecording()
    except Exception as exc:
        print(f"Warning stopping recording: {exc}")

    try:
        mtw_manipulator.stopMeasurement()
    except Exception as exc:
        print(f"Warning stopping measurement: {exc}")

    elapsed = time.perf_counter() - start_time

    print("\n========================================")
    print("           RECORDING COMPLETE")
    print("========================================")
    print(f"Samples recorded: {sample_count}")
    print(f"Dataset saved to: {os.path.abspath(output_file)}")
    print(f"Elapsed time: {elapsed:.2f} s")

    if elapsed > 0:
        print(
            f"Average sample rate: "
            f"{sample_count / elapsed:.2f} Hz"
        )

    print("\nPackets received from each sensor:")

    for sensor in sensor_names:
        print(f"{sensor}: {packet_counts[sensor]}")
