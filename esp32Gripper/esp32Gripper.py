# code operating the gripper.


# Run straight through terminal, with Ethernet connection to the UR5e arms (and compute box)

from flask import Flask, render_template_string, request
from onrobot import RG      # https://github.com/takuya-ki/onrobot-rg/tree/main
import time
from rtde_receive import RTDEReceiveInterface as RTDEReceive






	
robot_right = RTDEReceive('192.168.0.183', 125, [], True, False, 90)  # Adjust IP of UR5e here
robot_left = RTDEReceive('192.168.0.222', 125, [], True, False, 90)

app = Flask(__name__)
rg_right = RG('rg6', '192.168.0.232', 502)      # Adjust IP of grippers here
rg_right.set_target_force(100)                  # in 1/10th Newtons
rg_left = RG('rg6', '192.168.0.101', 502)
rg_left.set_target_force(100)
print(robot_right.getSafetyStatusBits())        # Check validity of status bits
print(robot_left.getSafetyStatusBits())

# Shared variable indicating whether to keep closing
keep_closing_right = False
keep_closing_left = False
leftForce = 0
rightForce = 0

# The following bits indicate any type of emergency/protective stop of the arms themselves when read from the SafetyStatusBits registers
E_STOP_MASK = (
    (1 << 2) |  # protective stop
    (1 << 4) |  # safeguard stop
    (1 << 5) |  # system emergency stop
    (1 << 6) |  # robot emergency stop
    (1 << 7) |  # any emergency stop
    (1 << 10)   # stopped due to safety
)

def is_estop(stat_bit):                 # Returns true if the status bits indicate an e-stop state
    return bool(stat_bit & E_STOP_MASK)

def gripper_loop():     # Main loop for gripper - mainly required for continuous closing behaviour, and e-stop checking
    """Central loop that repeatedly closes the gripper if the flag is set."""
    global keep_closing_right
    global keep_closing_left
    
    while True:
        if(is_estop(robot_right.getSafetyStatusBits())):
            rg_right.set_control_mode(8)
            keep_closing_right = False
            print("RIGHT ESTOP")
        elif(is_estop(robot_left.getSafetyStatusBits())):
            rg_left.set_control_mode(8)
            print("LEFT ESTOP")
            keep_closing_left = False
        else:
            # Effectively tells the gripper to close continuously, rather than waiting for task completion. Bypasses issue of re-opening
            if rg_right and keep_closing_right:
                print("Closing Right")
                rg_right.set_control_mode(1)
                rg_right.close_gripper(leftForce)
            if rg_left and keep_closing_left:
                print("CLosing Left")
                rg_left.set_control_mode(1)
                rg_left.close_gripper(rightForce)
        time.sleep(0.1)  # small delay to avoid CPU hogging

def closeRight(force):
    global keep_closing_right
    keep_closing_right = True
    rightForce = force
    return 'Closing right started'

def closeRight(force):
    global keep_closing_left
    keep_closing_left = True
    rightForce = force
    return 'Closing left started'

def openRight():
    global keep_closing_right
    keep_closing_right = False  # stop the repeated close
    if rg_right and not rg_right.get_status()[0]:
        rg_right.move_gripper(1200)
        while rg_right.get_status()[0]:  # wait until motion finishes
            time.sleep(0.1)
    return 'Gripper opened'
    
def openLeft():
    global keep_closing_left
    keep_closing_left = False  # stop the repeated close
    if rg_left and not rg_left.get_status()[0]:
        rg_left.move_gripper(1200)
        while rg_left.get_status()[0]:  # wait until motion finishes
            time.sleep(0.1)
    return 'Gripper opened'


def gripDetectedRight():
    return 'maybe'

def gripDetectedLeft():
    return 'maybe'