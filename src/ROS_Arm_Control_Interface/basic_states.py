from __future__ import annotations
from typing import Callable, List

import rospy

from .state import Machine, State
from .controller import Controller


# Basic States Categories:

# AUTOMATIC ARM CONTROL
# VR MANUAL ARM CONTROL
# CONDITIONALS


# AUTOMATIC ARM CONTROL

class Wait(State):
    _start_time: rospy.Time

    def __init__(self, machine: Machine) -> None:
        super().__init__(machine)
        self.wait_time = 100000
        
    def enter(self) -> None:
        print("Current State: " + str(self.name))
        self._start_time = rospy.Time.now()


    def run(self) -> None | State:
        elapsed_time = (rospy.Time.now() - self._start_time).to_sec()
        if elapsed_time >= self.wait_time:
            return self.to['next']
        return None

class GotoPosition(State):
    point: List[float]
    _start_time: rospy.Time

    def __init__(self, machine: Machine) -> None:
        super().__init__(machine)
        self.wait_time = 2
        self._has_arrived = False
        self.point = None
    
    def enter(self) -> None:
        print("Current State: " + str(self.name))
        self.machine.controller.automatic_mode()
        self.machine.controller.set_target_position_ee(None, self.point)
        self._start_time = rospy.Time.now()
        self._has_arrived = False

    def run(self) -> State | None:
        if self.machine.controller.has_arrived_target():
            if not self._has_arrived:
                self._start_time = rospy.Time.now()
                self._has_arrived = True
            elapsed_time = (rospy.Time.now() - self._start_time).to_sec()
            if elapsed_time >= self.wait_time:
                return self.to['next']
        return None
    
class GotoPositionRel(State):
    point: List[float]
    _start_time: rospy.Time

    def __init__(self, machine: Machine) -> None:
        super().__init__(machine)
        self.wait_time = 2
        self._has_arrived = False
        self.point = None
    
    def enter(self) -> None:
        print("Current State: " + str(self.name))
        self.machine.controller.automatic_mode()
        self.machine.controller.set_target_position_ee(self.point)
        self._start_time = rospy.Time.now()
        self._has_arrived = False

    def run(self) -> State | None:
        if self.machine.controller.has_arrived_target():
            if not self._has_arrived:
                self._start_time = rospy.Time.now()
                self._has_arrived = True
            elapsed_time = (rospy.Time.now() - self._start_time).to_sec()
            if elapsed_time >= self.wait_time:
                return self.to['next']
        
        return None

class UseGripper(State):
    width: float
    speed: float
    force: int
    _start_time: rospy.Time
    def __init__(self, machine: Machine) -> None:
        super().__init__(machine)
        self.wait_time = 3
        self._has_arrived = False
        self.width = self.machine.controller.rg_pose_default
        self.speed = self.machine.controller.rg_speed_default
        self.force = self.machine.controller.rg_force_default
        self.debug_time = 10
    
    def enter(self) -> None:
        print("Current State: " + str(self.name) + str(self.width))
        self.machine.controller.set_target_position_rg(self.width, self.speed, self.force)
        self.machine.controller.gripper_mode()
        self._start_time = rospy.Time.now()
        self._has_arrived = False

    def run(self) -> State | None:
        if self.machine.controller.has_gripped_target():
            if not self._has_arrived:
                self._start_time = rospy.Time.now()
                self._has_arrived = True
            elapsed_time = (rospy.Time.now() - self._start_time).to_sec()
            if elapsed_time >= self.wait_time:
                return self.to['next']
        elapsed_time = (rospy.Time.now() - self._start_time).to_sec()
        if elapsed_time >= self.debug_time:
            print(self.machine.controller.get_controller_info())
            return self.to['next']
        return None

class MoveGripper(UseGripper):
    def __init__(self, machine: Machine) -> None:
        super().__init__(machine)
    def enter(self) -> None:
        super().enter()
        self.machine.controller.publish_rg_move()
    
class GraspGripper(UseGripper):
    def __init__(self, machine: Machine) -> None:
        super().__init__(machine)
    def enter(self) -> None:
        super().enter()
        self.machine.controller.publish_rg_grasp()

class StopGripper(UseGripper):
    def __init__(self, machine: Machine) -> None:
        super().__init__(machine)
    def enter(self) -> None:
        super().enter()
        self.machine.controller.publish_rg_stop()
    def run(self) -> None:
        elapsed_time = (rospy.Time.now() - self._start_time).to_sec()
        if elapsed_time >= self.wait_time:
            return self.to['next']
        return None


class ResetDefault(State):
    _start_time: rospy.Time

    def __init__(self, machine: Machine) -> None:
        super().__init__(machine)
        self.wait_time = 5
        self._has_arrived = False
        self.point = None
    
    def enter(self) -> None:
        print("Current State: " + str(self.name))
        self.machine.controller.automatic_mode()
        self.machine.controller.set_default_ee_position()
        self._start_time = rospy.Time.now()
        self._has_arrived = False

    def run(self) -> State | None:
        if self.machine.controller.has_arrived_target():
            if not self._has_arrived:
                self._start_time = rospy.Time.now()
                self._has_arrived = True
            elapsed_time = (rospy.Time.now() - self._start_time).to_sec()
            if elapsed_time >= self.wait_time:
                return self.to['next']
        return None

class Start(ResetDefault):
    def __init__(self, machine: Machine) -> None:
        super().__init__(machine)
        

class Finish(State):
    """Lands drone

    Sends land command, disarms the drone and waits for MAVROS to signal grounded state

    """
    has_disarmed: bool

    def __init__(self, machine: Machine) -> None:
        super().__init__(machine)
        self.is_terminal = True

    def enter(self) -> None:
        print("Current State: " + str(self.name))

    def run(self) -> None:
        self.machine.finished = True

# CONDITIONALS


class Branch(State):
    """Runs an expression and sends next state from return value

    Attributes:
        expression (Callable[[], str | None]): an expression with no arguments that returns either a string, or None.
        If a string is returned Branch will use it as the key in the self.to dict to find a State to return.
        If None is received Branch will return None, and run again next cycle
    """
    expression: Callable[[], str | None]
    
    def __init__(self, machine: Machine) -> None:
        super().__init__(machine)

    def run(self) -> State | None:
        result = self.expression()
        if isinstance(result, str):
            return self.to[result]
        return None


class Counter(Branch):
    """Counts to an int, then returns a different state.

    Set to['if_repeat'] as the next State if counter is less than count_to

    Set to['if_count'] as the next State once counter reaches count_to

    Attributes:
        count_to (int): value to count to
    """

    def __init__(self, machine: Machine) -> None:
        super().__init__(machine)
        self.expression = self._counter
        self.count_to = 0
        self._count = 0

    def _counter(self) -> str:
        if self._count >= self.count_to:
            self._count = 0  # you can't use enter() or exit() because Counter must preserve internal state between reenters
            return 'if_count'
        self._count += 1
        return 'if_repeat'

    def exit(self) -> None:
        self.logger(f"{self._count}/{self.count_to}")


# VR MANUAL ARM CONTROL
class VRControl(Branch):
    '''
    States:
    0. Currently in this state, all buttons are not pressed, waiting for a command
    1. if Grab is hit, then the controller will move to controlling position, where it will move the robot arm to track the controller cube
    2. if Index is hit, then the controller will move to controlling the gripper.
        2a. Move the joystick up or down to adjust for the target width. Default will be set for both grasp and move
            2a i. If Index is hit again, the controller will enter the grasping action, and try to grasp whatever object it can.
            2a ii. If the stick is hiht, the controller will enter the moving action, and will move to the target width
    3. if x_input is hit, everything will reset to default. if x_input is hit 3 times in a row, the controller will shut down.
    4. if y_input is hit, start an autonomous mission that should wind back to the controller. 
        4a. hitting y_input during autonomous mission should end the mission prematurely.
    '''

    def __init__(self, machine: Machine) -> None:
        super().__init__(machine)
        self.expression = self._button_input
        self.exit_on = 3
        self._exit_count = 0
    def _button_input(self) -> str:
        if self.machine.controller.button_input.x_input:
            if self._exit_count <= 0:
                self._exit_count += 1
                return 'reset_default'
            if self._exit_count >= self.exit_on:
                self._exit_count = 0  # you can't use enter() or exit() because Counter must preserve internal state between reenters
                return 'if_exit'
            else:
                self._exit_count += 1
                return 'waiting_vr_control'
        if self.machine.controller.button_input.y_input:
            self._exit_count = 0
            return 'autonomous_mission'
        if self.machine.controller.button_input.grab:
            self._exit_count = 0
            return 'position_control'
        if self.machine.controller.button_input.index:
            self._exit_count = 0
            return 'gripper_control'        
        return None
    def enter(self):
        print("Current State: " + str(self.name) + " | " + str(self._exit_count) + "/" + str(self.exit_on))
        states_description = """
        0. Currently in this state, all buttons are not pressed, waiting for a command
        1. if Grab is hit, then the controller will move to controlling position, where it will move the robot arm to track the controller cube
        2. if Index is hit, then the controller will move to controlling the gripper.
            2a. Move the joystick up or down to adjust for the target width. Default will be set for both grasp and move
                2a i. If Index is hit again, the controller will enter the grasping action, and try to grasp whatever object it can.
                2a ii. If the stick is hit, the controller will enter the moving action, and will move to the target width
        3. if x_input is hit, everything will reset to default. if x_input is hit 3 times in a row, the controller will shut down.
        4. if y_input is hit, start an autonomous mission that should wind back to the controller. 
            4a. hitting y_input during autonomous mission should end the mission prematurely.
        """
        self.logger(states_description)

class ResetToDefaultVR(State):
    _start_time: rospy.Time

    def __init__(self, machine: Machine) -> None:
        super().__init__(machine)
        self.wait_time = 5
        self._has_arrived = False
    
    def enter(self) -> None:
        print("Current State: " + str(self.name))
        self.machine.controller.automatic_mode()
        self.machine.controller.set_default_ee_position()
        self._start_time = rospy.Time.now()
        self._has_arrived = False
        self.logger(str("Resetting to Default Position"))

    def run(self) -> State | None:
        if self.machine.controller.has_arrived_target():
            if not self._has_arrived:
                self._start_time = rospy.Time.now()
                self._has_arrived = True
            elapsed_time = (rospy.Time.now() - self._start_time).to_sec()
            if elapsed_time >= self.wait_time:
                return self.to['next']
        return None
    
    def exit(self) -> None:
        self.machine.controller.set_target_position_rg()
        self.machine.controller.gripper_mode()
        self.machine.controller.publish_rg_move()
        self.machine.controller.automatic_mode()
        self.machine.controller.init_cube_controller()
        
class CubeControl(State):
    _start_time: rospy.Time

    def __init__(self, machine: Machine) -> None:
        super().__init__(machine)
    
    def enter(self) -> None:
        print("Current State: " + str(self.name))
        self.machine.controller.manual_position_mode()
        self.machine.controller.set_target_position_cc()
        self._start_time = rospy.Time.now()
        self._has_arrived = False
        self.logger(str("Release Grab to go back to control \n" + 
                           "Move Cube to move end-effector" ))

    def run(self) -> State | None:
        self.machine.controller.set_target_position_cc()
        if not self.machine.controller.button_input.grab:
            return self.to['next']
        return None

class GripperControl(Branch):
    def __init__(self, machine: Machine) -> None:
        super().__init__(machine)
        self.expression = self._gripper_mode
        self.start_time = 1.5

    def _gripper_mode(self) -> str:
        if self.machine.controller.button_input.x_input:
            self.machine.controller.publish_rg_stop()
            return 'waiting_vr_control'
        if self.machine.controller.button_input.stick:
            return 'move_gripper'
        if self.machine.controller.button_input.index:
            return 'grasp_gripper'
        return None
    def enter(self) -> None:
        self._start_time = rospy.Time.now()
        print("Current State: " + str(self.name))
        self.logger(str("X_input to go back to control \n" + 
                        "Stick to move gripper \n" + 
                        "Index to grasp gripper \n" +
                        "Stick up to increase width \n" +
                        "Stick down to decrease width \n"))
        elapsed_time = (rospy.Time.now() - self._start_time).to_sec()
        if elapsed_time <= self.start_time:
            rospy.sleep(1.5) 
    def run(self) -> State | None:
        if self.machine.controller.button_input.stick_up:
            current_target = self.machine.controller.rg_pose_target
            if current_target < 0.045:
                self.machine.controller.set_target_position_rg(current_target+0.001)
        if self.machine.controller.button_input.stick_down:
            current_target = self.machine.controller.rg_pose_target
            if current_target > 0.001:
                self.machine.controller.set_target_position_rg(current_target-0.001)
        result = self.expression()
        if isinstance(result, str):
            return self.to[result]
        return None
    def exit(self) -> None:
        self.machine.controller.set_target_position_rg(self.machine.controller.rg_pose_target)

class GripperControlUse(State):
    def __init__(self, machine: Machine) -> None:
        super().__init__(machine)
        self.wait_time = 0.1
        self.start_time = 1.5
        self.debug_time = 10
        self._has_arrived = False

    def enter(self) -> None:
        print("Current State: " + str(self.name))
        self.machine.controller.gripper_mode()
        self._start_time = rospy.Time.now()
        self._has_arrived = False
    
    def run(self) -> State |None:
        if self.machine.controller.button_input.x_input:
            self.machine.controller.publish_rg_stop()
            return self.to['next']
        if self.machine.controller.has_gripped_target():
            if not self._has_arrived:
                self._start_time = rospy.Time.now()
                self._has_arrived = True
            elapsed_time = (rospy.Time.now() - self._start_time).to_sec()
            if elapsed_time >= self.wait_time:
                return self.to['next']
        elapsed_time = (rospy.Time.now() - self._start_time).to_sec()
        if elapsed_time >= self.debug_time:
            print(self.machine.controller.get_controller_info())
            return self.to['next']
        return None
        
    
class GripperControlMove(GripperControlUse):
    def __init__(self, machine: Machine) -> None:
        super().__init__(machine)
    
    def enter(self) -> None:
        super().enter()
        self.logger(str("Moving Gripper"))
        self.machine.controller.publish_rg_move()
        

class GripperControlGrasp(GripperControlUse):
    def __init__(self, machine: Machine) -> None:
        super().__init__(machine)
    
    def enter(self) -> None:
        super().enter()
        self.logger(str("Grasping Gripper"))
        self.machine.controller.publish_rg_grasp()
    

