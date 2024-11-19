from wind_turbine_inspection.states.base import InspectionState, WindTurbineInspectionStage
from std_msgs.msg import String

class ReturnHomeState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('return_home_state', WindTurbineInspectionStage.RETURN_HOME, state_machine)
        self.returnHomePublisher = self.create_publisher(String, '/drone_control/go_home', 10)
        self.returnHomePublisher.publish(String(data="1"))

    def waypoint_reached_callback(self, msg):
        self.get_logger().info("Inspection ended correctly")
        self.advance_to_next_state()
