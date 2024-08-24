from wind_turbine_inspection.states.base import InspectionState, WindTurbineInspectionStage

class OrthogonalAlignmentState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('orthogonal_alignment_state', WindTurbineInspectionStage.ORTHOGONAL_ALIGNMENT, state_machine)
        self.timer = self.create_timer(5.0, self.startAfter5Seconds)
        self.get_logger().info(f"Timer created {self.timer.time_until_next_call()}")

    def startAfter5Seconds(self):
        self.advance_to_next_state()


    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"OrthogonalAlignmentState received: {msg.data}")
