# class controlling an onRobot RG6 gripper.
import time
import threading


from onrobot import RG      # https://github.com/takuya-ki/onrobot-rg/tree/main
from rtde_receive import RTDEReceiveInterface as RTDEReceive

MAX_WIDTH = 2300
MIN_WIDTH = 800

class RG6gripper:
    
    E_STOP_MASK = (
        (1 << 2) |  # protective stop
        (1 << 4) |  # safeguard stop
        (1 << 5) |  # system emergency stop
        (1 << 6) |  # robot emergency stop
        (1 << 7) |  # any emergency stop
        (1 << 10)   # stopped due to safety
    )

    def __init__(self, robotIP = "10.18.248.40", rgIP = "10.18.248.41"):
        self.robot_right = RTDEReceive(robotIP, 125, [], True, False, 90)  # Adjust IP of UR5e here

        self.rg_right = RG('rg6', rgIP, 502)      # Adjust IP of grippers here
        self.rg_right.set_target_force(100)  


        # print(robot_right.getSafetyStatusBits())        # Check validity of status bits

        self.new_right = False                     # state variables

        self.right_force = 0

        self.right_possition = MAX_WIDTH


        self._running = True                                # gripper thread

        self._gripper_thread = threading.Thread(
            target = self.gripper_loop,
            daemon = True
        )

        self._gripper_thread.start()


    def is_estop(self, stat_bit):                 # Returns true if the status bits indicate an e-stop state
        return bool(stat_bit & self.E_STOP_MASK)


    def check_safety_right(self):                                   # check right and left arms
        return self.is_estop(self.robot_right.getSafetyStatusBits())

    def check_safety_left(self):
        return self.is_estop(self.robot_left.getSafetyStatusBits())


    def gripper_loop(self):
        # self.rg_right.set_control_mode(1)
        while self._running:

            if self.check_safety_right():                   # right arm check
                self.rg_right.set_control_mode(8)
                # self.right_possition = MAX_WIDTH

            elif self.rg_right and self.new_right:
                self.new_right = False    
                self.rg_right.move_gripper(self.right_possition, self.right_force)


            time.sleep(0.1)



    def close_right(self, force):           #right arm
        if self.check_safety_right():
            return False
        self.right_force = force
        self.keep_closing_right = True
        return True


    def open_right(self):
        self.keep_closing_right = False  # stop the repeated close

        if self.rg_right and not self.rg_right.get_status()[0]:
            self.rg_right.move_gripper(1200)
            while self.rg_right.get_status()[0]:  # wait until motion finishes
                time.sleep(0.1)
            return True
        else:
            return False


    def move_right(self, width = 1, force = 100):
        new_position = MIN_WIDTH + int(width * (MAX_WIDTH - MIN_WIDTH))
        if self.check_safety_right():
            return False
        self.right_force = force
        if new_position < 0:
           new_position = 0
        elif new_position > MAX_WIDTH:
            new_position = MAX_WIDTH
        if abs(new_position - self.right_possition) > 5:
            self.right_possition = new_position
            self.new_right = True 
        return True





    def gripDetectedRight(self):
            return bool(self.rg_right._read_register(268) & 0x0002)


    def shutdown(self):
        self.keep_closing_right = False
        self.keep_closing_left = False
        self._running = False

