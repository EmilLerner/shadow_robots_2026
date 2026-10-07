import xsensdeviceapi as xda
import csv
import time
import os

from wirelessMasterCallback import WirelessMasterCallback
from mtwManipulator import MtwManipulator


# ============================================================
# SETTINGS
# ============================================================

UPDATE_RATE = 80
RADIO_CHANNEL = 19

OUTPUT_FILE = "imu_dataset.csv"


# Actual sensor IDs detected from your Xsens system
SENSOR_IDS = {
    "S2": "00B4F199",
    "S3": "00B4F744",
    "S4": "00B4F130"
}


# ============================================================
# CSV COLUMNS
# ============================================================

columns = [
    "timestamp",

    "S2_ACC_X",
    "S2_ACC_Y",
    "S2_ACC_Z",
    "S2_GYRO_X",
    "S2_GYRO_Y",
    "S2_GYRO_Z",

    "S3_ACC_X",
    "S3_ACC_Y",
    "S3_ACC_Z",
    "S3_GYRO_X",
    "S3_GYRO_Y",
    "S3_GYRO_Z",

    "S4_ACC_X",
    "S4_ACC_Y",
    "S4_ACC_Z",
    "S4_GYRO_X",
    "S4_GYRO_Y",
    "S4_GYRO_Z"
]


# ============================================================
# CONNECT TO XSENS
# ============================================================

print("\nConnecting to Xsens...")

control = xda.XsControl.construct()

wireless_master_callback = WirelessMasterCallback()

mtw_callbacks = {}

mtw_manipulator = MtwManipulator(
    control,
    wireless_master_callback,
    UPDATE_RATE,
    RADIO_CHANNEL,
    mtw_callbacks
)


# ============================================================
# SCAN FOR DEVICES
# ============================================================

print("\nScanning for Xsens devices...")

mtw_manipulator.scanXsDevices()

print("Starting configuration mode...")

mtw_manipulator.startConfigMode()

print("Waiting for sensor connections...")

mtw_manipulator.waitForConnections()

print("Starting measurement...")

mtw_manipulator.startMeasurement()

print("Resetting orientation data...")

mtw_manipulator.resetOrientationData()

print("Starting recording...")

mtw_manipulator.startRecording()


# ============================================================
# CHECK CALLBACKS
# ============================================================

print("\nCallbacks available:")

print(mtw_callbacks)


# ============================================================
# OPEN CSV
# ============================================================

file_exists = os.path.exists(OUTPUT_FILE)

csv_file = open(
    OUTPUT_FILE,
    "a",
    newline=""
)

writer = csv.writer(csv_file)

if not file_exists:
    writer.writerow(columns)


# ============================================================
# RECORDING
# ============================================================

print("\n========================================")
print("        IMU DATA COLLECTION")
print("========================================")
print("Move your arm naturally.")
print("Press Ctrl+C to stop recording.\n")


start_time = time.perf_counter()

sample_count = 0


try:

    while True:

        # ----------------------------------------------------
        # Data from each sensor
        # ----------------------------------------------------

        sensor_data = {
            "S2": None,
            "S3": None,
            "S4": None
        }


        # ----------------------------------------------------
        # Read all available packets
        # ----------------------------------------------------

        for sensor_id, callback in mtw_callbacks.items():

            while callback.dataAvailable():

                packet = callback.getOldestPacket()

                device_id = str(packet.deviceId())

                # --------------------------------------------
                # Identify sensor
                # --------------------------------------------

                sensor_name = None

                for name, configured_id in SENSOR_IDS.items():

                    if device_id == configured_id:
                        sensor_name = name
                        break


                if sensor_name is None:

                    callback.deleteOldestPacket()

                    continue


                # --------------------------------------------
                # Accelerometer
                # --------------------------------------------

                acc = packet.calibratedAcceleration()

                acc_x = float(acc[0])
                acc_y = float(acc[1])
                acc_z = float(acc[2])


                # --------------------------------------------
                # Gyroscope
                # --------------------------------------------

                gyro = packet.calibratedGyroscopeData()

                gyro_x = float(gyro[0])
                gyro_y = float(gyro[1])
                gyro_z = float(gyro[2])


                sensor_data[sensor_name] = [
                    acc_x,
                    acc_y,
                    acc_z,
                    gyro_x,
                    gyro_y,
                    gyro_z
                ]


                callback.deleteOldestPacket()


        # ====================================================
        # SAVE ONLY WHEN ALL THREE SENSORS HAVE NEW DATA
        # ====================================================

        if (
            sensor_data["S2"] is not None
            and sensor_data["S3"] is not None
            and sensor_data["S4"] is not None
        ):

            timestamp = time.perf_counter() - start_time


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

                # -------------------------
                # S3
                # -------------------------

                sensor_data["S3"][0],
                sensor_data["S3"][1],
                sensor_data["S3"][2],
                sensor_data["S3"][3],
                sensor_data["S3"][4],
                sensor_data["S3"][5],

                # -------------------------
                # S4
                # -------------------------

                sensor_data["S4"][0],
                sensor_data["S4"][1],
                sensor_data["S4"][2],
                sensor_data["S4"][3],
                sensor_data["S4"][4],
                sensor_data["S4"][5]
            ]


            writer.writerow(row)

            csv_file.flush()

            sample_count += 1


            # ----------------------------------------------
            # Display progress every second
            # ----------------------------------------------

            if sample_count % UPDATE_RATE == 0:

                elapsed = time.perf_counter() - start_time

                actual_rate = sample_count / elapsed

                print(
                    f"Samples: {sample_count:6d} | "
                    f"Time: {elapsed:6.1f} s | "
                    f"Rate: {actual_rate:6.1f} Hz"
                )


        # ----------------------------------------------------
        # Small delay to prevent excessive CPU usage
        # ----------------------------------------------------

        time.sleep(0.001)


# ============================================================
# STOP
# ============================================================

except KeyboardInterrupt:

    print("\n\nStopping recording...")


finally:

    csv_file.close()

    try:
        mtw_manipulator.stopRecording()
    except:
        pass

    try:
        mtw_manipulator.stopMeasurement()
    except:
        pass

    print("\n========================================")
    print("           RECORDING COMPLETE")
    print("========================================")

    print(f"Samples recorded: {sample_count}")

    print(f"Dataset saved to: {OUTPUT_FILE}")

    if sample_count > 0:

        elapsed = time.perf_counter() - start_time

        print(
            f"Average sample rate: "
            f"{sample_count / elapsed:.2f} Hz"
        )