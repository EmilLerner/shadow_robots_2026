import time
import struct
import serial
import rtde_receive

ROBOT_IP = "192.168.1.102"
SERIAL_PORT = "COM3"  # Change to your port (e.g. /dev/ttyACM0)

rtde = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
ser = serial.Serial(SERIAL_PORT, 115200)
time.sleep(1.5)  # Allow CDC to settle

header = bytes([0xAA, 0x55])

try:
    while True:
        # Index 2 = Elbow joint torque (Nm)
        elbow_torque = float(rtde.getActualMoment()[2])

        # Send [0xAA, 0x55, 4-byte float]
        ser.write(header + struct.pack('<f', elbow_torque))
        print(f"\rTorque: {elbow_torque:+6.2f} Nm", end="", flush=True)

        time.sleep(0.02)  # 50 Hz

except KeyboardInterrupt:
    # Safe cutoff on exit
    ser.write(header + struct.pack('<f', 0.0))
    ser.close()
    rtde.disconnect()
    print("\nStopped.")