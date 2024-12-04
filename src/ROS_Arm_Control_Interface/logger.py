from __future__ import annotations

import threading
from colorama import Fore, Back, Style

import rospy

from std_msgs.msg import String, Float64

from . import topics


class ThreadedPublisher:
    """Base class for creating publishers on another thread
    """

    def __init__(self, rate=20, init_node=False) -> None:
        """Create a threaded publisher

        Args:
            rate (int, optional): rate to send at (hz). Defaults to 20.
            init_node (bool, optional): self init a ros node. Defaults to False.
        """
        # we auto set "name" and the actual ros node name based on class name
        self.name = type(self).__qualname__
        self.node_name = "".join(['_' + c.lower() if c.isupper() else c for c in self.name]).lstrip('_')
        if init_node:
            rospy.init_node(self.node_name)

        self._rate = rospy.Rate(rate)

    def publish(self) -> None:
        """Abstract Method

        Function to call every self.rate, usually put the pub.publish() here

        """
        pass

    def exit(self) -> bool:
        """Abstract Method

        Check if the thread should exit or not, useful if you are tying the ThreadedPublisher to an existing node

        Returns:
            bool: True to exit thread
        """
        return False

    def start(self) -> None:
        """Start the thread
        """
        print(f"L: Starting {self.name}")
        thread = threading.Thread(target=self._cycle)
        thread.start()

    def _cycle(self) -> None:
        while not rospy.is_shutdown():
            if self.exit():
                break
            self.publish()
            self._rate.sleep()

        print(Fore.BLUE, end='')
        print(f"L: {self.name} thread exited")
        print(Style.RESET_ALL, end='')


