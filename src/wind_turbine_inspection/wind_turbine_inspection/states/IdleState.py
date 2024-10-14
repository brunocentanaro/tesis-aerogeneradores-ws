from wind_turbine_inspection.states.base import InspectionState, WindTurbineInspectionStage


class IdleState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('idle_state', WindTurbineInspectionStage.IDLE, state_machine)
        self.declare_parameter('mission_param', 0)
        self.declare_parameter('front_inspection', 1)
        mission_param = self.get_parameter(
            'mission_param').get_parameter_value().integer_value
        isFrontInspectionParam = self.get_parameter(
            'front_inspection').get_parameter_value().integer_value == 1
        self.update_shared_state('mission_param', mission_param)
        self.update_shared_state('is_front_inspection', isFrontInspectionParam)
        self.get_logger().info(f"Front inspection: {isFrontInspectionParam}")

        self.get_logger().info(f"Mission parameter: {mission_param}")
        self.get_logger().info("IdleState created")
        self.srv = self.create_service(Trigger, 'comenzar_inspeccion', self.comenzar_inspeccion_callback)
        self.get_logger().info(
            f"Timer created {self.timer.time_until_next_call()}")

    def comenzar_inspeccion_callback(self, request, response):
        self.advance_to_next_state()
        response.success = True
        response.message = "Inspección iniciada"
        return response

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"IdleState received: {msg.data}")
