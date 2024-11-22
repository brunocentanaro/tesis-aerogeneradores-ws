from wind_turbine_inspection.states.base import InspectionState, WindTurbineInspectionStage
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition, FailsafeFlags
import rclpy

class IdleState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('idle_state', WindTurbineInspectionStage.IDLE, state_machine)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=100
        )

        self.declare_parameter('mission_param', 0)
        self.declare_parameter('front_inspection', 1)
        mission_param = self.get_parameter(
            'mission_param').get_parameter_value().integer_value
        isFrontInspectionParam = self.get_parameter(
            'front_inspection').get_parameter_value().integer_value == 1
        
        # Update the shared state with mission and inspection type parameters.
        self.update_shared_state('mission_param', mission_param) # indicates what turbine we want to inspect
        self.update_shared_state('is_front_inspection', isFrontInspectionParam) # indicates if its a front inspection
        self.get_logger().info(f"Front inspection: {isFrontInspectionParam}")

        self.get_logger().info(f"Mission parameter: {mission_param}")
        self.get_logger().info("IdleState created")
        self.srv = self.create_service(
            Trigger,
            'start_inspection',
            self.start_inspection_callback)
        self.invalidFlags = None

        self.failsafeSub = self.create_subscription(
            FailsafeFlags,
            '/fmu/out/failsafe_flags',
            self.failsafeFlagsCallback,
            qos_profile)

    # Callback function to handle failsafe flags.
    # It checks various conditions and adds them to the invalidFlags list.
    def failsafeFlagsCallback(self, msg):
        checks = [
            'angular_velocity_invalid',
            'attitude_invalid',
            'local_position_invalid',
            'global_position_invalid',
            'offboard_control_signal_lost',
            'home_position_invalid',
            'battery_low_remaining_time',
            'battery_unhealthy',
            'local_position_accuracy_low',
            'fd_critical_failure',
            'fd_imbalanced_prop',
            'fd_motor_failure'
        ]
        self.invalidFlags = []
        for check in checks:
            if getattr(msg, check):
                self.invalidFlags.append(check)

    # Callback para salir del IdleState, verifica las fallas del sistema y avanza al siguiente estado si no hay problemas.
    def start_inspection_callback(self, request, response):
        if self.invalidFlags is None:
            response.success = False
            response.message = "Status unknown"
            return response
        if len(self.invalidFlags) > 0:
            response.success = False
            response.message = f"Invalid flags: {self.invalidFlags}"
            return response

        self.advance_to_next_state()
        response.success = True
        response.message = "Inspección iniciada"
        return response

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"IdleState received: {msg.data}")
