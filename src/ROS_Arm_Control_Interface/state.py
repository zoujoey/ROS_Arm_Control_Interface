from __future__ import annotations
from typing import List, Optional
from enum import Enum

from .controller import Controller
from .types import StateMode


class Machine:
    """Acts as a wrapper for a collection of states

    See examples for creating a StateMachine with States

    StateMachine should not run anything on init! (e.g. fetch data from ROS topic) Use create() or on_enter()

    Any ControlStack must call StateMachine methods in correct order and time

    Raises:
        RuntimeError: if no start_state is given

    Attributes:
        controller (Controller): controller the machine can control
        start_state (State): state to run on start/restart
        current_state (State | None): current state in the cycle
        previous_state (State | None): last state, None if current is first state
        name (str): defaults to class name
        description (str): defaults to class name
        embedded (bool): if the machine is embedded in at least one other machine
        finished (bool): internal flag to signal completion
        state_list (List[States]): list of states in machine in chronological order of instantiation

    Methods:
        abstract create -> None: called when creating machine, for actions that may require Controller to be initialized, etc.
        abstract on_enter -> None: called when starting or restarting machine
        abstract on_exit -> None: called when the machine is exited
        add_state -> None: add a state to the state_list
        cycle -> None: iterate machine in the event loop
        enter -> None: called when starting or restarting machine
        embed -> State: embed a StateMachine as a state
    """
    start_state: State

    def __init__(self, controller: Controller) -> None:
        """Creates a StateMachine instance
        """
        self.controller = controller
        self.name = type(self).__qualname__
        self.description = type(self).__qualname__
        self.embedded = False
        self.finished = False
        self._current_state: State | None = None
        self._previous_state: State | None = None

        self.state_list: List[State] = []

    @property
    def current_state(self) -> State | None:
        return self._current_state

    @property
    def previous_state(self) -> State | None:
        return self._previous_state

    def create(self) -> None:
        """Abstract Method

        Called once in control stack before event loop starts, or once by the Embed constructor

        """
        return None

    def on_enter(self) -> None:
        """Abstract Method
        
        Called when machine is started/restarted

        """
        return None

    def on_exit(self) -> None:
        """Abstract Method

        Called when the machine is exited

        """
        return None

    def add_state(self, state: State) -> None:
        """Add state to state_list, mutates state name if name is not unique

        Args:
            state (State): state to add to state_list
        """
        # Ensure name is unique, not essential but makes debugging easier
        for s in self.state_list:
            if s.name == state.name:
                state.name = state.name + "*"

        self.state_list.append(state)  # add state to Machine list

    def cycle(self) -> None:
        """Iterates the state machine
        """
        if self._current_state is None:
            print("Current State is None Somehow")
        else:
            next_state = self._current_state._cycle()  # pylint: disable=protected-access
            if isinstance(next_state, State):
                self._previous_state = self._current_state
                self._current_state = next_state

    def enter(self) -> None:
        """Called when machine is started/restarted

        Non abstract version of enter, calls abstract on_enter() in body

        You probably want to implement on_enter() if you require custom StateMachine actions

        Raises:
            RuntimeError: if no start_state is given
        """
        if self.start_state is None:
            raise RuntimeError("No start state")
        self.finished = False
        self._current_state = self.start_state
        self._previous_state = None
        self.on_enter()

    def exit(self) -> None:
        """Called when machine is started/restarted

        Non abstract version of exit, calls abstract on_exit() in body

        You probably want to implement on_exit() if you require custom StateMachine actions
        """
        self.finished = True
        self._current_state = None
        self._previous_state = None
        self.on_exit()

    def embed(self, machine: Machine) -> State:
        """Embed a machine as a state

        Args:
            machine (StateMachine): StateMachine to embed

        Returns:
            State: machine as a state
        """
        return Embed(self, machine)


class State:
    """State

    State should not run anything on init! (e.g. fetch data from ROS topic) Use enter()/run()/exit()

    `to['next']` should be used as the default key for the "next state" in non branching States, and the key for "default

    Standard naming convention for branching states is:

    `to['if_foobar']`, `to['when_foobar']` where foobar is descriptive name.

    See examples for creating custom States.

    Attributes:
        name (str): defaults to class name
        description (str): defaults to class name
        contains_embed (bool): if State is a wrapper of an embedded StateMachine
        embed (StateMachine | None): StateMachine wrapped by state
        is_terminal (bool): mark if the state can terminate the machine, used for logging
        termination_comment (str): reason for state terminating the machine, used for logging
        machine (StateMachine): the machine State is a part of
        to (dict[str, State]): dictionary providing States to transition to, use 'next' as default key

    Methods:
        abstract enter -> (State | None): called once if the State was not running before
        abstract run -> (State | None): always called in event loop
        abstract exit -> (State | None): if run() returns a State instance then exit() is called before transitioning to another State

    Developer Notes:

    For logging purposes each state should have a unique name in its assigned machine, this can be changed if you refactor the graphing and logging to use something like python's builtin id().
    """
    to: dict[str, State]

    def __init__(self, machine: Machine) -> None:
        """Creates a State instance

        StateMachine instance is passed in constructor rather than by inheritance in order to reduce lines of code needed when implementing a Machine.

        ```
        self.state = State()
        self.add_state(state)
        ```

        vs.

        ```
        self.state = State(self)
        ```
        Args:
            machine (StateMachine): the machine the State is a part of
        """
        self.name = type(self).__qualname__
        self.description = type(self).__qualname__
        self.contains_embed = False
        self.embed: Machine | None = None
        self.is_terminal = False
        self.termination_comment = ""
        self.machine = machine
        self.to = {}
        self._status = StateMode.CREATED

        self.machine.add_state(self)

    def enter(self) -> State | None:
        """Abstract Method

        Called when state is entered/re-entered

        Returns:
            next (State | None): if State instance is returned, machine will transition to given State
        """
        return None

    def run(self) -> State | None:
        """Abstract Method

        Called every event loop iteration, any long running blocking action, or action that should be preformed autonomously should be a separate thread or ROS node.

        Returns:
            next (State | None): if State instance is returned, machine will transition to given State
        """
        return None

    def exit(self) -> State | None:
        """Abstract Method

        Called when state is exited (i.e. run() or enter() returns a State instance)

        exit() has no effect on terminal states such as "Land" as they never return a State

        Returns:
            next (State | None): if State instance is returned, machine will transition to given State
        """
        return None

    def logger(self, message: str) -> None:
        """Send a message to the state info ROS topic

        Uses the `state_logger_pub` on the parent machine's controller

        Args:
            message (str): message to send
        """
        self.machine.controller.state_logger_pub.publish(name=self.name, id=str(id(self)), message=message)

    def _cycle(self) -> State | None:
        """Runs state as a part of the event loop

        Method evaluation is as follows:
        1. enter() is called once if the State was not running before
        2. run() is always called in event loop
        3. if run() returns a State instance then exit() is called before transitioning to another State

        All methods enter(), run(), and exit() are capable of returning a State instance. The priority each of these returned States are given is:
        1. exit()
        2. run()
        3. enter()

        Returns:
            next (State | None): if State instance is returned, machine will transition to given State
        """
        enter_return = None
        run_return = None
        exit_return = None

        if self._status == StateMode.WAITING or self._status == StateMode.CREATED:
            enter_return = self.enter()  # pylint: disable=assignment-from-none
            self._status = StateMode.RUNNING

        run_return = self.run()  # pylint: disable=assignment-from-none

        if isinstance(run_return, State):
            exit_return = self.exit()  # pylint: disable=assignment-from-none
            self._status = StateMode.WAITING

        if isinstance(exit_return, State):
            return exit_return
        if isinstance(run_return, State):
            return run_return
        if isinstance(enter_return, State):
            return enter_return

        return None


class Embed(State):
    """A StateMachine embedded in a state

    Returns 'next' key after embedded machine has exited

    Attributes:
        embed (StateMachine): the embedded state machine

    """

    def __init__(self, machine: Machine, embed: Machine) -> None:
        """Embed a StateMachine as a state

        Args:
            machine (StateMachine): the machine the embed State is a part of
            embed (StateMachine): machine instance to embed
        """
        super().__init__(machine)

        self.embed = embed

        # TODO entering hack job territory here with how we ensure names are unique, don't want to fully override constructor though...
        self.name = self.embed.name + " Embedded"
        # Ensure name is unique, not essential but might make debugging easier
        for s in self.machine.state_list:
            if s is self:
                continue
            if s.name == self.name:
                self.name = self.name + "*"

        self.embed.embedded = True
        self.contains_embed = True
        self.embed.create()

    def enter(self) -> None:
        if self.embed is not None:
            self.embed.enter()
        else:
            print("Embed embed is None somehow")

    def run(self) -> State | None:
        if self.embed is not None:
            self.embed.cycle()
            if self.embed.finished:
                if 'next' in self.to:
                    return self.to['next']
                else:
                    self.machine.finished = True
        else:
            print("Embed embed is None somehow")
        return None
