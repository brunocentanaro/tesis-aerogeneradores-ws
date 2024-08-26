from wind_turbine_inspection.states.base import InspectionState, WindTurbineInspectionStage

class BackInspectionState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('back_inspection_state', WindTurbineInspectionStage.BACK_INSPECTION, state_machine)

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"BackInspectionState received: {msg.data}")
