from wirelessMasterCallback import WirelessMasterCallback
from mtwManipulator import MtwManipulator
from pidController import PIDcontroller
from mpcControllerV5 import MPCController
from armMotionWithMPC import ArmMotion
from scipy.optimize import minimize

import sys
import time

import numpy as np
import inverse_kinematics_solver4 as iks

from math import pi

import xsensdeviceapi as xda
from scipy.spatial.transform import Rotation as R

import rtde_control, rtde_receive
import time
import csv

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

    arm_dims = [0.375, 0.3, 0.075] # shoulder, forearm, hand
    arm_factor = 1.1
    arm_dims = [dim * arm_factor for dim in arm_dims]

    desired_update_rate = 80
    desired_radio_channel = 19

    mtws = {
        "00B4F130": "SHOU R",
        "00B4F1B4": "fARM R",
        "00B4F198": "HAND R"
    }

    ### CONFIGURABLE FIELDS ###

    mtws_right = {k: v for k, v in mtws.items() if v.endswith("R")}

    right_arm = ArmMotion(arm_dims, "NONE", mtws) # Configurable second parameter

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
