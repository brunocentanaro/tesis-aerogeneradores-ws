import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from abc import ABC, abstractmethod
from enum import Enum
from rclpy.task import Future

class WindTurbineInspectionStage(Enum):
    IDLE = "idle"
    TAKEOFF = "takeoff"
    APPROACH = "approach"
    ORTHOGONAL_ALIGNMENT = "orthogonal_alignment"
    INSPECTION = "inspection"
    RETURN_HOME = "return_home"

class InspectionState(ABC, Node):
    shared_state = {}

    def __init__(self, node_name, name, state_machine):
        super().__init__(node_name)
        self.name = name
        self.state_machine = state_machine
        self.subscriber = self.create_subscription(String, '/drone_control/waypoint_reached', self.waypoint_reached_callback, 10)
        self._future = Future()

    @abstractmethod
    def waypoint_reached_callback(self, msg):
        pass

    def advance_to_next_state(self):
        self._future.set_result(True)
    
    def get_future(self):
        return self._future

    @classmethod
    def update_shared_state(cls, key, value):
        cls.shared_state[key] = value

