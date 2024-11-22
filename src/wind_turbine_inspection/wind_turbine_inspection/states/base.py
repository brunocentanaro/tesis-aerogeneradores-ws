import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from abc import ABC, abstractmethod
from enum import Enum
from rclpy.task import Future

# Enum that defines the different stages of the wind turbine inspection process
class WindTurbineInspectionStage(Enum):
    IDLE = "idle"  # The drone is idle and not performing any action
    TAKEOFF = "takeoff"  # The drone is taking off
    APPROACH = "approach"  # The drone is approaching the wind turbine
    ORTHOGONAL_ALIGNMENT = "orthogonal_alignment"  # The drone aligns orthogonally with the turbine
    INSPECTION = "inspection"  # The drone is inspecting the wind turbine
    RETURN_HOME = "return_home"  # The drone is returning to the starting point

class InspectionState(ABC, Node):
    shared_state = {} # A class-level dictionary that holds shared state across instances

    def __init__(self, node_name, name, state_machine):
        super().__init__(node_name)
        self.name = name
        self.state_machine = state_machine
        # Subscription to listen for messages indicating when a waypoint is reached
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

