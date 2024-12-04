from __future__ import annotations
import os
import threading
import rospy
from colorama import Fore, Back, Style

from ROS_Arm_Control_Interface.msg import StackInfo, StackEvent  # pylint: disable=no-name-in-module,import-error
from std_msgs.msg import String

from .state import Machine, State
from .controller import Controller
from .logger import ThreadedPublisher
from .types import ControlMode, StackMode
from . import topics


class ControlStack:
    """The ControlStack class integrates the ControlController and StateMachine.

    Handles initialization of arm and state:
    1. Passes arm_controller to state_machine
    2. Handles state_machine.create()
    3. Initializes connection to arm and enters state machine
    5. Preforms any initial checks
    6. Starts the event loop

    Handles the event loop:
    1. State machine is run
        a. If machine flags finished the event loop is terminated
    2. Repeat

    Raises:
        RuntimeError: raised on initialization error

    Attributes:
        machine (StateMachine): must be provided by composition
        arm_controller (armController): must be provided by composition
        name (str): ControlStack name

    Methods:
        start -> None: initializes and starts event loop

    """
    machine: Machine
    controller: Controller


    def __init__(self, name="Control_Stack") -> None:
        """Creates ROS node and ControlStack instance

        Args:
            name (str, optional): ROS node name. Defaults to "Control_Stack".
        """
        rospy.init_node(name)
        self.name = name

        self._rate = rospy.Rate(20)

        self.mode = StackMode.INITIAL #Initialize Stack Mode
        self._info_publisher = StateInfoPublisher(self) #Control
        self._event_publisher = rospy.Publisher(topics.STACK_EVENT, StackEvent, queue_size=0)
        self._exit = False  # tell info publisher thread to exit on True

    def start(self) -> None:
        """Starts the ControlStack, handles all creation/initialization logic
        """
        self._create()
        self._begin_control()
        self._run()

    def _create(self) -> None:
        """Called when starting the ControlStack, handles create() instance members

        Raises:
            RuntimeError: No state_machine passed
            RuntimeError: No arm_controller passed
        """
        print(Fore.RED, end='')
        if self.machine is None:
            raise RuntimeError("No state_machine")
        if self.controller is None:
            raise RuntimeError("No arm_controller")
        print(Style.RESET_ALL, end='')

        # Create graph if server is enabled
        
        self.mode = StackMode.START
        self._send_stack_event()
        self.machine.create()

    def _begin_control(self) -> None:
        """Called before entering event loop, handles arm and state machine initialization
        """
        self.controller.initialize_connection()
        self.machine.enter()
        self._info_publisher.start()
        self._pre_control_check()

    def _pre_control_check(self) -> None:
        """Handles precontrol checks

        Currently no precontrol checks exists, if any are added they should raise a RuntimeError on fail

        """
        return None

    def _run(self) -> None:
        """The event loop, terminates on state machine exit
        """
        print("CS: Begin Event Loop")
        self.mode = StackMode.RUNNING
        self._send_stack_event()
        while not rospy.is_shutdown():
            self.machine.cycle()

            if self.machine.finished is True:
                print(Fore.BLUE, end='')
                print(f"CS: Top level machine {self.machine.name} exit")
                print(Style.RESET_ALL, end='')
                break

            self._rate.sleep()  # sleep in order to not hog system resources

            self._control_updates()

        self.machine.controller.exit()
        self._exit = True
        self.mode = StackMode.EXIT
        self._send_stack_event()
        print("CS: Event Loop Exit")

    def _control_updates(self) -> None:
        """Calls control updates in the event loop

        Currently does nothing, target publishing and logging have been moved to separate threads

        """
        return

    def _send_stack_event(self) -> None:
        """Send a control event, currently just indicates the status (start and exit) of the stack itself
        """
        self._event_publisher.publish(stack_status=self.mode.value)


class StateInfoPublisher(ThreadedPublisher):
    """Parallelize control stack info publishing
    """

    def __init__(self, control_stack: ControlStack, rate=20) -> None:
        super().__init__(rate)
        self.control_stack = control_stack
        self._pub = rospy.Publisher(topics.STACK_INFO, StackInfo, latch=True, queue_size=10)

    def exit(self) -> bool:
        return self.control_stack._exit

    def publish(self) -> None:
        stack_info = StackInfo()

        # TODO: schema of representing current state is needed
        # currently we just get the current state on the top level machine, but embedded machines are a single state, thus making it difficult to determine what "current" state to send
        # also at some point we probably want to support showing embedded machines to a set depth, so it has to play nice with that
        stack_info.current_state = self.get_state_name(self.control_stack.machine.current_state) or ""
        stack_info.current_state_id = self.get_state_id(self.control_stack.machine.current_state) or ""
        stack_info.previous_state = self.get_state_name(self.control_stack.machine.previous_state) or ""
        stack_info.pervious_state_id = self.get_state_id(self.control_stack.machine.previous_state) or ""

        self._pub.publish(stack_info)

    def get_state_name(self, state: State | None) -> str | None:
        """Get the name of a State

        Args:
            state (State | None): state to get the name of

        Returns:
            str | None: the name, None if passed None
        """
        if state is not None:
            if state.contains_embed:
                # TODO not quite sure what the cost of calling the name finder recursively every cycles is, optimize if needed later
                return self._get_name_embed(state)[3:]
            else:
                return state.name
        return None

    def _get_name_embed(self, state: State, name="") -> str:
        """Get state name recursively if it embeds a state machine
        """
        if state.contains_embed:
            if state.embed is not None:
                if state.embed.current_state is not None:
                    return name + " | " + state.name + " | " + self._get_name_embed(state.embed.current_state, name)
        return state.name

    def get_state_id(self, state: State | None) -> str | None:
        """Get the unique id of a State

        Args:
            state (State | None): state to get the name of

        Returns:
            str | None: the id, None if passed None
        """
        if state is not None:
            if state.contains_embed:
                return self._get_id_embed(state)
            else:
                return str(id(state))
        return None

    def _get_id_embed(self, state: State) -> str:
        """Get state id recursively if it embeds a state machine
        """
        if state.contains_embed:
            if state.embed is not None:
                if state.embed.current_state is not None:
                    return self._get_id_embed(state.embed.current_state)
        return str(id(state))
