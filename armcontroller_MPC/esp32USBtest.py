from espHeptics import espHeptics
import time

ROBOT_IP_RIGHT = "192.168.1.183" # REAL IP, DON'T USE FOR TESTING AND DEBUGGING

RG_IP_RIGHT = "192.168.1.1"

EMS_torque_frequency = 0

heptics = espHeptics(robotIP=ROBOT_IP_RIGHT,rgIP=RG_IP_RIGHT, torqueFrequency=EMS_torque_frequency)



while True:
    time.sleep(1)  # Delay 1 second
heptics.close()