from wind_turbine_inspection.states.base import InspectionState, WindTurbineInspectionStage
from wind_turbine_inspection.states.constants import windTurbineTypeAndLocation
from std_msgs.msg import String

class TakeoffState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('takeoff_state', WindTurbineInspectionStage.TAKEOFF, state_machine)

        windTurbineHeight = windTurbineTypeAndLocation[self.shared_state['mission_param']]['height']
        self.startTakeoffPublisher = self.create_publisher(String, 'start_takeoff_procedure', 10)
        self.startTakeoffPublisher.publish(String(data=f"{windTurbineHeight}"))

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"TakeoffState received: {msg.data}")
        self.advance_to_next_state()