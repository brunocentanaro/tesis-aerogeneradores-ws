import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from abc import ABC, abstractmethod
from enum import Enum
import threading
import time

windTurbineTypeAndLocation = [
    {
        "height": 50,
        "coordinates": {
            "latitude": -34.627257,
            "longitude": -54.957857
        },
        "bladeLength": 20
    }
]


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
        self.publisher = self.create_publisher(String, 'state_topic', 10)
        self.subscriber = self.create_subscription(String, 'waypoint_reached', self.waypoint_reached_callback, 10)

    @abstractmethod
    def is_finished(self):
        pass

    @abstractmethod
    def next_state(self):
        pass

    @abstractmethod
    def waypoint_reached_callback(self, msg):
        pass

    def advance_to_next_state(self):
        next_state = self.next_state()
        self.state_machine.change_state(next_state)

    @classmethod
    def update_shared_state(cls, key, value):
        cls.shared_state[key] = value

class IdleState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('idle_state', WindTurbineInspectionStage.IDLE, state_machine)
        self.declare_parameter('mission_param', 0)
        mission_param = self.get_parameter('mission_param').get_parameter_value().integer_value
        self.update_shared_state('mission_param', mission_param)

        self.get_logger().info(f"Mission parameter: {mission_param}")
        self.get_logger().info("IdleState created")
        self.timer = self.create_timer(20.0, self.startAfter20Seconds)
        self.get_logger().info(f"Timer created {self.timer.time_until_next_call()}")

    def startAfter20Seconds(self):
        self.advance_to_next_state()

    def is_finished(self):
        return True

    def next_state(self):
        return TakeoffState(self.state_machine)

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"IdleState received: {msg.data}")

class TakeoffState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('takeoff_state', WindTurbineInspectionStage.TAKEOFF, state_machine)

        self.startTakeoffPublisher = self.create_publisher(String, 'start_takeoff_procedure', 10)
        self.startTakeoffPublisher.publish(String(data="start"))

    def is_finished(self):
        return True

    def next_state(self):
        return ApproachState(self.state_machine)

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"TakeoffState received: {msg.data}")
        self.advance_to_next_state()

class ApproachState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('approach_state', WindTurbineInspectionStage.APPROACH, state_machine)
        self.publisher = self.create_publisher(String, 'gps_waypoint', 10)
        mission_param = self.shared_state['mission_param']

        self.get_logger().info('Publishing waypoint')
        newCoord = windTurbineTypeAndLocation[mission_param]['coordinates']
        height = windTurbineTypeAndLocation[mission_param]['height']
        self.publish_waypoint(f"{newCoord['latitude']},{newCoord['longitude']},{height}") 

    
    def publish_waypoint(self, waypoint):
        msg = String()
        msg.data = waypoint
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def is_finished(self):
        return True

    def next_state(self):
        return OrthogonalAlignmentState(self.state_machine)

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"ApproachState received: {msg.data}")
        self.advance_to_next_state()


class OrthogonalAlignmentState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('orthogonal_alignment_state', WindTurbineInspectionStage.ORTHOGONAL_ALIGNMENT, state_machine)
        self.advance_to_next_state()

    def is_finished(self):
        return True

    def next_state(self):
        return FrontInspectionState(self.state_machine)

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"OrthogonalAlignmentState received: {msg.data}")

class FrontInspectionState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('front_inspection_state', WindTurbineInspectionStage.FRONT_INSPECTION, state_machine)
        windTurbineId = self.shared_state['mission_param']
        windTurbineBladeLength = windTurbineTypeAndLocation[windTurbineId]['bladeLength']

        self.startInspectionPublisher = self.create_publisher(String, 'inspect_wind_turbine', 10)
        self.startInspectionPublisher.publish(String(data=f"{windTurbineBladeLength}"))

    def is_finished(self):
        return True

    def next_state(self):
        return BackInspectionState(self.state_machine)

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"FrontInspectionState received: {msg.data}")

class BackInspectionState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('back_inspection_state', WindTurbineInspectionStage.BACK_INSPECTION, state_machine)

    def is_finished(self):
        return True

    def next_state(self):
        return ReturnHomeState(self.state_machine)

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"BackInspectionState received: {msg.data}")

class ReturnHomeState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('return_home_state', WindTurbineInspectionStage.RETURN_HOME, state_machine)

    def is_finished(self):
        return True

    def next_state(self):
        return IdleState()

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"ReturnHomeState received: {msg.data}")
    

class WindTurbineInspectionStateMachine(Node):
    def __init__(self):
        super().__init__('inspection_state_machine')
        self.current_state = IdleState(self)
        rclpy.spin_once(self.current_state)
        self.run()

    def change_state(self, new_state):
        self.current_state.destroy_node()
        self.current_state = new_state
        self.get_logger().info(f"State changed to: {self.current_state.name}")
        rclpy.spin_once(self.current_state) 

    def run(self):
        rclpy.spin(self)

# Inicialización de ROS 2
def main(args=None):
    rclpy.init(args=args)

    state_machine = WindTurbineInspectionStateMachine()

    rclpy.spin(state_machine)

    state_machine.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()