from wind_turbine_inspection.states.base import InspectionState, WindTurbineInspectionStage
from std_srvs.srv import Trigger

class IdleState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('idle_state', WindTurbineInspectionStage.IDLE, state_machine)
        self.declare_parameter('mission_param', 0)
        mission_param = self.get_parameter('mission_param').get_parameter_value().integer_value
        self.update_shared_state('mission_param', mission_param)

        self.get_logger().info(f"Mission parameter: {mission_param}")
        self.get_logger().info("IdleState created")
        self.srv = self.create_service(Trigger, 'comenzar_inspeccion', self.comenzar_inspeccion_callback)

    def comenzar_inspeccion_callback(self, request, response):
        self.advance_to_next_state()
        response.success = True
        response.message = "Inspección iniciada"
        return response

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"IdleState received: {msg.data}")
