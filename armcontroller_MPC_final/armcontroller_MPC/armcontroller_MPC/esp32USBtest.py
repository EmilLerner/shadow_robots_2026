from espHeptics import espHeptics
import time

ROBOT_IP_RIGHT = "10.18.248.40" # REAL IP, DON'T USE FOR TESTING AND DEBUGGING

RG_IP_RIGHT = "10.18.248.41"

EMS_torque_frequency = 10

heptics = espHeptics(robotIP=ROBOT_IP_RIGHT,rgIP=RG_IP_RIGHT, torqueFrequency=EMS_torque_frequency)


time.sleep(100)
heptics.close()