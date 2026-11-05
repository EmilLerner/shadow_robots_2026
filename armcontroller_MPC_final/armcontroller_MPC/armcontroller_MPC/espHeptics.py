# sample code for python

import serial
import threading
import time
import rtde_receive

from RG6gripper import RG6gripper




class espHeptics:
    def __init__(self, port = "COM6", baudrate = 115200, robotIP = "10.18.248.40", rgIP = "10.18.248.41", torqueFrequency = 0):
        #esp32 set up
        self.gripper = RG6gripper(robotIP, rgIP)
        self.rtde = rtde_receive.RTDEReceiveInterface(robotIP)
        # print([
        #     m for m in dir(self.rtde)
        #     if "moment" in m.lower()
        # ])
        self.ser = serial.Serial(port, baudrate, timeout=1)

        # set up torue intervals
        self.torqueF = torqueFrequency
        if self.torqueF != 0:
            self.torque_interval = 1 / self.torqueF
            self.torque_update_time = time.monotonic()
            self.prev = 2
            self.alpha = 0.1

        time.sleep(2)
        # set up a thread
        self.run = True
        # self.lock = threading.Lock()
        self.thread = threading.Thread(target = self.receive, daemon = True)
        self.thread.start()



  


    def send_message(self, message: str):
        self.ser.write((message + "\n").encode("utf-8"))
        # print(f"Sent: {message}")


    def receive(self):
        while self.run:
            try:
                if self.torqueF != 0:
                    current_time = time.monotonic()

                    if (current_time  - self.torque_update_time) > self.torque_interval:
                        # gather torque
                        elbow_torque = self.filterTorue(self.rtde.getActualCurrent()[2])

                        # Send torque back through ESP1
                        self.send_message(f"ELBOWTORQUE,{elbow_torque:.2f}")
                        self.torque_update_time = current_time

                # recive teh vales 
                line = self.ser.readline().decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                # print(f"Received: {line}")
                # splits the messege into header and parts

                parts = line.split(",", 1)
                header = parts[0]

                # the header is GRIP and the lenght matches
                if header == "GRIP" and len(parts) == 2:
                    try:
                        value = float(parts[1])

                        # Execute the grip action
                        self.gripper.move_right(value, 1200)

                        # Check the resulting grip state
                        status = bool(self.gripper.gripDetectedRight())

                        # Send Boolean result back through ESP1
                        self.send_message(f"GRIPSTATUS,{int(status)}")

                        
                    except ValueError:
                        # print("Invalid GRIP float value")
                        b = 5

            except serial.SerialException as e:
                print(f"Serial error: {e}")
                break

    def close(self):
        self.run = False
        self.thread.join(timeout = 2)
        self.ser.close()

    def filterTorue(self, current):

        self.prev = (self.alpha * current + (1 - self.alpha) * self.prev)
        torque = (self.prev - 1.8) * 15
        return max(-20, min(20, torque))
        