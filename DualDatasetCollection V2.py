import xsensdeviceapi as xda
import csv
import time
import os
import msvcrt
import DualVariableConfig

from wirelessMasterCallback import WirelessMasterCallback
from mtwManipulator import MtwManipulator


# ============================================================
# SETTINGS
# ============================================================

update_rate = DualVariableConfig.SAMPLE_RATE
radio_channel = DualVariableConfig.RADIO_CHANNEL
raw_filename = DualVariableConfig.RAW_FILENAME

# Maximum allowed difference between sensor packet arrival times.
max_time_skew = DualVariableConfig.MAX_TIME_SKEW

# Sensor mapping
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


# ============================================================
# CSV COLUMN HEADERS
# ============================================================

dataset_cols = ["timestamp"]

for sensor, side in [
    ("S1", "R"),
    ("S2", "R"),
    ("S3", "R"),
    ("S4", "L"),
    ("S5", "L"),
    ("S6", "L")
]:
    dataset_cols.extend([
        f"{sensor}_ACC_X_{side}",
        f"{sensor}_ACC_Y_{side}",
        f"{sensor}_ACC_Z_{side}",
        f"{sensor}_GYRO_X_{side}",
        f"{sensor}_GYRO_Y_{side}",
        f"{sensor}_GYRO_Z_{side}",
        f"{sensor}_QUAT_W_{side}",
        f"{sensor}_QUAT_X_{side}",
        f"{sensor}_QUAT_Y_{side}",
        f"{sensor}_QUAT_Z_{side}"
    ])


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
# SCAN AND CONFIGURE DEVICES
# ============================================================

print("\nScanning for Xsens devices...")
mtw_manipulator.scanXsDevices()

print("\nStarting configuration mode...")
mtw_manipulator.startConfigMode()

print("\nWaiting for sensor connections...")
mtw_manipulator.waitForConnections()


# ============================================================
# START MEASUREMENT
# ============================================================

print("\nStarting measurement...")
mtw_manipulator.startMeasurement()

print("\nResetting orientation data...")
mtw_manipulator.resetOrientationData()

print("\nStarting recording...")
mtw_manipulator.startRecording()


# ============================================================
# PREPARE DATA COLLECTION
# ============================================================

sensor_names = ("S1", "S2", "S3", "S4", "S5", "S6")

# Store all complete samples in memory.
# Each item in this list will become one CSV row.
recorded_data = []

sample_count = 0

packet_counts = {
    sensor: 0 for sensor in sensor_names
}

# Most recent IMU values for each sensor.
sensor_data = {
    sensor: None for sensor in sensor_names
}

# Track whether each sensor has provided a new packet.
updated_since_save = {
    sensor: False for sensor in sensor_names
}


# ============================================================
# DATA COLLECTION
# ============================================================

print("\n========================================")
print("          IMU DATA COLLECTION")
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
print("Press 's' to stop and save the CSV.")
print("Alternatively, press Ctrl+C to stop and save.\n")


start_time = time.perf_counter()
last_report_time = start_time
last_report_count = 0

try:

    while True:

        # ----------------------------------------------------
        # CHECK FOR S KEY WITHOUT BLOCKING DATA COLLECTION
        # ----------------------------------------------------

        if msvcrt.kbhit():
            key = msvcrt.getwch()

            if key.lower() == "s":
                print("\n\n'S' pressed. Stopping collection...")
                break

        # ----------------------------------------------------
        # READ AVAILABLE SENSOR PACKETS
        # ----------------------------------------------------

        callbacks = mtw_manipulator.getMtwCallbacks()

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
                        float(quaternion[3])
                    ]

                    sensor_data[bodypart] = values
                    updated_since_save[bodypart] = True
                    packet_counts[bodypart] += 1

                finally:
                    callback.deleteOldestPacket()

        # ----------------------------------------------------
        # CHECK WHETHER ALL SIX SENSORS HAVE DATA
        # ----------------------------------------------------

        all_ready = all(
            sensor_data[sensor] is not None
            for sensor in sensor_names
        )

        all_updated = all(
            updated_since_save[sensor]
            for sensor in sensor_names
        )

        # ----------------------------------------------------
        # STORE COMPLETE SAMPLE IN MEMORY
        # ----------------------------------------------------

        if all_ready and all_updated:

            timestamp = time.perf_counter() - start_time

            row = [timestamp]

            # Preserve the existing CSV column order.
            for sensor in sensor_names:
                row.extend(sensor_data[sensor])

            # Store in memory instead of writing to CSV.
            recorded_data.append(row)

            sample_count += 1

            # Each sensor must provide another packet
            # before the next sample is stored.
            updated_since_save = {
                sensor: False for sensor in sensor_names
            }

        # ----------------------------------------------------
        # REPORT COLLECTION PROGRESS
        # ----------------------------------------------------

        now = time.perf_counter()

        if now - last_report_time >= 2.0:

            elapsed = now - start_time
            interval = now - last_report_time

            new_samples = sample_count - last_report_count

            recent_rate = (
                new_samples / interval
                if interval > 0 else 0.0
            )

            average_rate = (
                sample_count / elapsed
                if elapsed > 0 else 0.0
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

        # Avoid excessive CPU usage when no packets are available.
        time.sleep(0.001)


except KeyboardInterrupt:
    print("\n\nCtrl+C pressed. Stopping collection...")


finally:

    # ========================================================
    # STOP XSENS BEFORE SAVING
    # ========================================================

    try:
        mtw_manipulator.stopRecording()
    except Exception as exc:
        print(f"Warning stopping recording: {exc}")


# ========================================================
# SAVE THE COMPLETE ARRAY TO CSV
# ========================================================

print("\nSaving collected samples to CSV...")

try:
    with open(raw_filename, "w", newline="") as csv_file:
        writer = csv.writer(csv_file)

        writer.writerow(dataset_cols)
        writer.writerows(recorded_data)

    print(f"CSV saved successfully to: {raw_filename}")

    # Clear the array after a successful save.
    recorded_data.clear()

    print("Recorded data array cleared from memory.")

except PermissionError:
    print(f"\nERROR: Cannot write to {raw_filename}.")
    print("Close the CSV in Excel or another program and retry.")
    print("Data remains in recorded_data.")

except OSError as exc:
    print(f"\nERROR saving CSV: {exc}")
    print("Data remains in recorded_data.")
    
    # ========================================================
    # FINAL SUMMARY
    # ========================================================

    elapsed = time.perf_counter() - start_time

    print("\n========================================")
    print("           RECORDING COMPLETE")
    print("========================================")

    print(f"Samples recorded: {sample_count}")
    print(f"Dataset rows: {len(recorded_data)}")
    print(f"Dataset columns: {len(dataset_cols)}")
    print(f"Elapsed time: {elapsed:.2f} s")
    print(f"Dataset path: {os.path.abspath(raw_filename)}")

    if elapsed > 0:
        print(
            f"Average sample rate: "
            f"{sample_count / elapsed:.2f} Hz"
        )

    print("\nPackets received from each sensor:")

    for sensor in sensor_names:
        print(f"{sensor}: {packet_counts[sensor]}")
