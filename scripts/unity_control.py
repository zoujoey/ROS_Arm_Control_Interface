#!/usr/bin/env python3

from ROS_Arm_Control_Interface.stack import ControlStack
from ROS_Arm_Control_Interface.state import Machine
from ROS_Arm_Control_Interface.controller import Controller
import ROS_Arm_Control_Interface.basic_states as BST


class Manual_Control(Machine):
    """Takes off and hovers for a little while, then lands.
    """

    def __init__(self, controller: Controller) -> None:
        super().__init__(controller)
        
        self.start = BST.Start(self)
        self.vr_control = BST.VRControl(self)
        self.reset_default = BST.ResetToDefaultVR(self)
        self.cc_control = BST.CubeControl(self)
        self.rg_control = BST.GripperControl(self)
        self.rgm_control = BST.GripperControlMove(self)
        self.rgg_control = BST.GripperControlGrasp(self)
        self.finish = BST.Finish(self)

        self.start.to['next'] = self.reset_default

        self.vr_control.to['reset_default'] = self.reset_default
        self.vr_control.to['waiting_vr_control'] = self.vr_control
        self.vr_control.to['autonomous_mission'] = self.finish
        self.vr_control.to['position_control'] = self.cc_control
        self.vr_control.to['gripper_control'] = self.rg_control
        self.vr_control.to['if_exit'] = self.finish

        self.reset_default.to['next'] = self.vr_control
        
        self.cc_control.to['next'] = self.vr_control

        self.rg_control.to['waiting_vr_control'] = self.vr_control
        self.rg_control.to['move_gripper'] = self.rgm_control
        self.rg_control.to['grasp_gripper'] = self.rgg_control
        
        self.rgm_control.to['next'] = self.vr_control
        self.rgg_control.to['next'] = self.vr_control

        self.start_state = self.start


if __name__ == "__main__":
    cs = ControlStack()
    ac = Controller()
    hover = Manual_Control(ac)

    cs.controller = ac
    cs.machine = hover

    cs.start()
