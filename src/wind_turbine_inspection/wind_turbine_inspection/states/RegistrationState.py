from wind_turbine_inspection.states.base import InspectionState, WindTurbineInspectionStage
from wind_turbine_inspection.states.constants import windTurbineTypeAndLocation
from std_msgs.msg import String

class RegistrationState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('registration_state', WindTurbineInspectionStage.INSPECTION, state_machine)
        bladeLength = windTurbineTypeAndLocation[self.shared_state['mission_param']]['bladeLength']
        rotorDiameter = windTurbineTypeAndLocation[self.shared_state['mission_param']]['rotorDiameter']
        self.startInspectionPublisher = self.create_publisher(String, '/drone_control/inspect_wind_turbine', 10)
        self.startInspectionPublisher.publish(String(data=f"{rotorDiameter},{bladeLength}"))

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"RegistrationState received: {msg.data}")
        self.state_machine.completedInspectionRounds += 1
        self.advance_to_next_state()