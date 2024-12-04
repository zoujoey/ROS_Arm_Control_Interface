from __future__ import annotations
from typing import NamedTuple, List
from enum import Enum
from dataclasses import dataclass


class InterfaceMode(NamedTuple): #Flight
    """InterfaceMode struct

    mode (str): raw control_mode param
    is_simulation (bool): is interface occurring in simulation? (mutually exclusive with is_real)
    is_hardware (bool): are we *testing* real hardware devices?
    is_real (bool): is the interface occurring in real life on the arm?
    """
    mode: str
    is_simulation: bool
    is_hardware: bool
    is_real: bool


class StateMode(Enum):
    """State class status enum
    """
    CREATED = 1  # Machine instance created, enter() has never been run
    WAITING = 2  # Machine is waiting to be reentered, enter() has been run at least once before
    RUNNING = 3  # Machine is currently calling run()


class StackMode(Enum):
    """Current mode of the ControlStack
    """
    INITIAL = 0  # Initialized by constructor
    START = 1    # ControlStack has preformed some checks and is preparing to enter the event loop
    RUNNING = 2  # ControlStack is running the event loop and Machine
    EXIT = 3     # ControlStack has exited


class ControlMode(Enum): #Drone
    """What type of movement the drone is actively preforming
    """
    MANUAL_POSITION = 1    # Manual position mode
    MANUAL_ORIENTATION = 2  # Manual pose mode
    AUTOMATIC = 3    # Automatic mode
    GRIPPER = 4       # Gripper mode
    FINISHED = 5      # Arm has reached end goal
    EXIT = 6        # Controller has ceded all control and is terminating

@dataclass
class InputButtonXR:
    grab: bool = False
    index: bool = False
    stick: bool = False
    x_input: bool = False
    y_input: bool = False
    stick_up: bool = False
    stick_down: bool = False