from wind_turbine_inspection.states.base import InspectionState, WindTurbineInspectionStage

class ReturnHomeState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('return_home_state', WindTurbineInspectionStage.RETURN_HOME, state_machine)

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"ReturnHomeState received: {msg.data}")
    
