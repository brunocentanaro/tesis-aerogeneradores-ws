from wind_turbine_inspection.states.base import InspectionState, WindTurbineInspectionStage

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

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"IdleState received: {msg.data}")
