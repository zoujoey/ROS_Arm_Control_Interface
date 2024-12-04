#!/usr/bin/env python3

from ROS_Arm_Control_Interface.stack import ControlStack
from ROS_Arm_Control_Interface.state import Machine
from ROS_Arm_Control_Interface.controller import Controller
import ROS_Arm_Control_Interface.basic_states as BST


class Auto_Move(Machine):
    """Takes off and hovers for a little while, then lands.
    """

    def __init__(self, controller: Controller) -> None:
        super().__init__(controller)
        
        self.start = BST.Start(self)
        self.position_rel = BST.GotoPositionRel(self)
        self.position = BST.GotoPosition(self)
        self.reset = BST.ResetDefault(self)
        self.finish = BST.Finish(self)
        self.move_grip = BST.MoveGripper(self)
        self.grasp_grip = BST.GraspGripper(self)
        self.stop_grip = BST.StopGripper(self)
        self.reset_grip = BST.MoveGripper(self)


        self.position_rel.point = [0.2, 0, 0.15, 0, 0, 0, 1]
        self.position.point = [0.505172 , -0.221972, 0.5, 1, 0, 0, 0]
        
        self.move_grip.width = 0.04
        self.move_grip.speed = 0.05

        self.grasp_grip.force = 5
        self.grasp_grip.width = 0.03
        self.grasp_grip.speed = 0.1
         
        self.reset_grip.width = 0.05
        self.reset_grip.speed = 0.05

        self.start.to['next'] = self.reset_grip
        self.reset_grip.to['next'] = self.position_rel
        self.position_rel.to['next'] = self.position
        self.position.to['next'] = self.move_grip
        self.move_grip.to['next'] = self.grasp_grip
        self.grasp_grip.to['next'] = self.stop_grip
        self.stop_grip.to['next'] = self.reset
        self.reset.to['next'] = self.finish
        self.start_state = self.start


if __name__ == "__main__":
    cs = ControlStack()
    ac = Controller()
    hover = Auto_Move(ac)

    cs.controller = ac
    cs.machine = hover

    cs.start()
