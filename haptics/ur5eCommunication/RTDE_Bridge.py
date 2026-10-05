import struct
import sys
import time
import rtde_receive
import serial

# --- Configuration ---
ROBOT_IP = "192.168.56.101"  # UR5e Controller IP
SERIAL_PORT = "/dev/ttyACM0"  # Transmitter ESP32 COM Port
BAUD_RATE = 115200
SAMPLE_RATE_HZ = 50  # Telemetry stream frequency (50 Hz = 20ms)

# Joint index: [0: Base, 1: Shoulder, 2: Elbow, 3: Wrist 1, 4: Wrist 2, 5: Wrist 3]
ELBOW_JOINT_INDEX = 2

# UR5e Size 3 joint torque constant (approx. 10.6 Nm/A for joint 2 / Elbow)
# Set to 1.0 if your receiving MCU already handles conversion or expects Amperes.
TORQUE_CONSTANT_NM_PER_A = 10.6


def main():
  print(f"[INIT] Connecting to UR5e RTDE at {ROBOT_IP}...")
  try:
    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
    print("[OK] Connected to UR5e.")
  except Exception as e:
    print(f"[ERROR] Failed to connect to UR5e: {e}")
    sys.exit(1)

  print(f"[INIT] Opening serial port {SERIAL_PORT} @ {BAUD_RATE}...")
  try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(1.5)  # Allow USB CDC initialization
    print("[OK] Serial bridge active.")
  except Exception as e:
    print(f"[ERROR] Could not open serial port: {e}")
    rtde_r.disconnect()
    sys.exit(1)

  print(
      "[RUNNING] Streaming raw elbow torque telemetry. Press Ctrl+C to stop."
  )
  frame_header = bytes([0xAA, 0x55])
  period = 1.0 / SAMPLE_RATE_HZ

  try:
    while rtde_r.isConnected():
      t_start = time.perf_counter()

      # Correct UR RTDE method: getActualCurrent() returns a list of 6 joint currents [A]
      actual_currents = rtde_r.getActualCurrent()
      elbow_current = actual_currents[ELBOW_JOINT_INDEX]

      # Convert current (A) -> torque estimate (Nm)
      elbow_torque = elbow_current * TORQUE_CONSTANT_NM_PER_A

      # Pack raw 32-bit float (little-endian '<f') -> 4 bytes
      payload = struct.pack("<f", elbow_torque)
      ser.write(frame_header + payload)

      elapsed = time.perf_counter() - t_start
      sleep_time = period - elapsed
      if sleep_time > 0:
        time.sleep(sleep_time)

  except KeyboardInterrupt:
    print("\n[STOPPING] Streaming 0.0 Nm torque baseline...")
  finally:
    try:
      ser.write(frame_header + struct.pack("<f", 0.0))
      time.sleep(0.05)
      ser.close()
    except Exception:
      pass
    rtde_r.disconnect()
    print("[DONE] Exited and connections closed.")


if __name__ == "__main__":
  main()