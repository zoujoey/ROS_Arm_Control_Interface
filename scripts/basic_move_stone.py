#!/usr/bin/env python3

from ROS_Arm_Control_Interface.stack import ControlStack
from ROS_Arm_Control_Interface.state import Machine
from ROS_Arm_Control_Interface.controller import Controller
import ROS_Arm_Control_Interface.basic_states as BST


class Auto_Move_Stone(Machine):
    """Takes off and hovers for a little while, then lands.
    """

    def __init__(self, controller: Controller) -> None:
        super().__init__(controller)
        
        self.start = BST.Start(self)
        self.position_rel = BST.GotoPositionRel(self)
        self.position_1 = BST.GotoPosition(self)
        self.position_2 = BST.GotoPosition(self)
        self.position_3 = BST.GotoPosition(self)
        self.position_4 = BST.GotoPosition(self)
        self.position_5 = BST.GotoPosition(self)
        self.position_6 = BST.GotoPosition(self)
        self.position_7 = BST.GotoPosition(self)
        self.position_8 = BST.GotoPosition(self)
        self.position_9 = BST.GotoPosition(self)
        self.position_10 = BST.GotoPosition(self)
        self.reset = BST.ResetDefault(self)
        self.finish = BST.Finish(self)
        
        self.move_grip = BST.MoveGripper(self)
        self.move_grip_2 = BST.MoveGripper(self)
        
        self.grasp_grip = BST.GraspGripper(self)
        self.grasp_grip_2 = BST.GraspGripper(self)

        self.stop_grip = BST.StopGripper(self)
        self.stop_grip_2 = BST.StopGripper(self)

        self.reset_grip = BST.MoveGripper(self)


        self.position_rel.point = [0.2, 0, 0.15, 0, 0, 0, 1]
        self.position_1.point = [0.505172 , -0.221972, 0.5, 1, 0, 0, 0]
        self.position_2.point = [0.505172 , -0.221972, 0.6, 1, 0, 0, 0]
        self.position_3.point = [0.505172 , 0.221972, 0.6, 1, 0, 0, 0]
        self.position_4.point = [0.505172 , 0.221972, 0.5, 1, 0, 0, 0]
        self.position_5.point = [0.505172 , 0.221972, 0.6, 1, 0, 0, 0]
        self.position_6.point = [0.505172 , 0.221972, 0.5, 1, 0, 0, 0]
        self.position_7.point = [0.505172 , 0.221972, 0.6, 1, 0, 0, 0]
        self.position_8.point = [0.505172 , -0.221972, 0.6, 1, 0, 0, 0]
        self.position_9.point = [0.505172 , -0.221972, 0.5, 1, 0, 0, 0]
        self.position_10.point = [0.505172 , -0.221972, 0.6, 1, 0, 0, 0]
        


        self.move_grip.width = 0.04
        self.move_grip.speed = 0.05

        self.move_grip_2.width = 0.04
        self.move_grip_2.speed = 0.05

        self.grasp_grip.force = 5
        self.grasp_grip.width = 0.03
        self.grasp_grip.speed = 0.1
         
        self.grasp_grip_2.force = 5
        self.grasp_grip_2.width = 0.03
        self.grasp_grip_2.speed = 0.1
        
        self.reset_grip.width = 0.04
        self.reset_grip.speed = 0.05

        self.start.to['next'] = self.reset_grip
        self.reset_grip.to['next'] = self.position_rel
        self.position_rel.to['next'] = self.position_1
        self.position_1.to['next'] = self.move_grip
        self.move_grip.to['next'] = self.grasp_grip
        self.grasp_grip.to['next'] = self.position_2
        self.position_2.to['next'] = self.position_3
        self.position_3.to['next'] = self.position_4
        self.position_4.to['next'] = self.stop_grip
        self.stop_grip.to['next'] = self.move_grip_2
        self.move_grip_2.to['next'] = self.position_5
        self.position_5.to['next'] = self.position_6
        self.position_6.to['next'] = self.grasp_grip_2
        self.grasp_grip_2.to['next'] = self.position_7
        self.position_7.to['next'] = self.position_8
        self.position_8.to['next'] = self.position_9
        self.position_9.to['next'] = self.stop_grip_2
        self.stop_grip_2.to['next'] = self.position_10
        self.position_10.to['next'] = self.reset
        self.reset.to['next'] = self.finish
        self.start_state = self.start


if __name__ == "__main__":
    cs = ControlStack()
    ac = Controller()
    hover = Auto_Move_Stone(ac)

    cs.controller = ac
    cs.machine = hover

    cs.start()
