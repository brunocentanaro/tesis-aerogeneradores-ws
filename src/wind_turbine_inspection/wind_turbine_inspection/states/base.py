import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from abc import ABC, abstractmethod
from enum import Enum

class WindTurbineInspectionStage(Enum):
    IDLE = "idle"
    TAKEOFF = "takeoff"
    APPROACH = "approach"
    ORTHOGONAL_ALIGNMENT = "orthogonal_alignment"
    FRONT_INSPECTION = "front_inspection"
    BACK_INSPECTION = "back_inspection"
    RETURN_HOME = "return_home"

class InspectionState(ABC, Node):
    shared_state = {}

    def __init__(self, node_name, name, state_machine):
        super().__init__(node_name)
        self.name = name
        self.state_machine = state_machine
        self.subscriber = self.create_subscription(String, '/drone_control/waypoint_reached', self.waypoint_reached_callback, 10)

    @abstractmethod
    def waypoint_reached_callback(self, msg):
        pass

    def advance_to_next_state(self):
        self.state_machine.change_state()

    @classmethod
    def update_shared_state(cls, key, value):
        cls.shared_state[key] = value

