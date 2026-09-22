# class controlling an onRobot RG6 gripper.
import time
import threading


from onrobot import RG      # https://github.com/takuya-ki/onrobot-rg/tree/main
from rtde_receive import RTDEReceiveInterface as RTDEReceive



class RG6gripper:
    
    E_STOP_MASK = (
        (1 << 2) |  # protective stop
        (1 << 4) |  # safeguard stop
        (1 << 5) |  # system emergency stop
        (1 << 6) |  # robot emergency stop
        (1 << 7) |  # any emergency stop
        (1 << 10)   # stopped due to safety
    )

    def __init__(self):
        self.robot_right = RTDEReceive('192.168.0.183', 125, [], True, False, 90)  # Adjust IP of UR5e here
        self.robot_left = RTDEReceive('192.168.0.222', 125, [], True, False, 90)

        self.rg_right = RG('rg6', '192.168.0.232', 502)      # Adjust IP of grippers here
        self.rg_left = RG('rg6', '192.168.0.101', 502)

        self.rg_right.set_target_force(100)  
        self.rg_left.set_target_force(100)

        # print(robot_right.getSafetyStatusBits())        # Check validity of status bits
        # print(robot_left.getSafetyStatusBits())

        self.keep_closing_right = False                     # state variables
        self.keep_closing_left = False
        self.left_force = 0
        self.right_force = 0


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
        while self._running:

            if self.check_safety_right():                   # right arm check
                self.rg_right.set_control_mode(8)
                self.keep_closing_right = False
                # print("RIGHT ESTOP")

            elif self.rg_right and self.keep_closing_right:      #activate right arm
                self.rg_right.set_control_mode(1)
                self.rg_right.close_gripper(self.right_force)
                

            if self.check_safety_left():                   # left arm check
                self.rg_left.set_control_mode(8)
                self.keep_closing_left = False
                # print("LEFT ESTOP")

            elif self.rg_left and self.keep_closing_left:      #activate left arm
                self.rg_left.set_control_mode(1)
                self.rg_left.close_gripper(self.left_force)

            time.sleep(0.1)



    def close_right(self, force):           #right arm
        if self.check_safety_right():
            return False
        self.right_force = force
        self.keep_closing_right = True

        return True

    def close_left(self, force):           #left arm
        if self.check_safety_left():
            return False
        self.left_force = force
        self.keep_closing_left = True

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


    def open_left(self):
        self.keep_closing_left = False  # stop the repeated close

        if self.rg_left and not self.rg_left.get_status()[0]:
            self.rg_left.move_gripper(1200)
            while self.rg_left.get_status()[0]:  # wait until motion finishes
                time.sleep(0.1)
            return True
        else:
            return False



    def gripDetectedRight(self):
        return False

    def gripDetectedLeft(self):
        return False

    def shutdown(self):
        self.keep_closing_right = False
        self.keep_closing_left = False
        self._running = False

