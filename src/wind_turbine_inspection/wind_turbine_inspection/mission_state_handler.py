import rclpy
from rclpy.node import Node
from wind_turbine_inspection.states.ApproachState import ApproachState
from wind_turbine_inspection.states.RegistrationState import RegistrationState
from wind_turbine_inspection.states.IdleState import IdleState
from wind_turbine_inspection.states.OrthogonalAlignmentState import OrthogonalAlignmentState
from wind_turbine_inspection.states.ReturnHomeState import ReturnHomeState
from wind_turbine_inspection.states.TakeoffState import TakeoffState
from px4_msgs.msg import VehicleLocalPosition, TrajectorySetpoint, BatteryStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from wind_turbine_inspection.TestResult import TestResult
from std_msgs.msg import String


class WindTurbineInspectionStateMachine(Node):
    def __init__(self):
        super().__init__('inspection_state_machine')
        self.declare_parameter('register_after_takeoff', 0)

        self.registerAfterTakeoff = self.get_parameter(
            'register_after_takeoff').get_parameter_value().integer_value

        self.registerAfterTakeoff = RegistrationState if self.registerAfterTakeoff == 1 else ApproachState
        self.current_state_publisher = self.create_publisher(
            String, '/inspection_state_machine/state', 10)
        self.current_state = IdleState(self)
        self.completedFirstRound = False
        self.spin_until_state_complete()

    def spin_until_state_complete(self):
        future = self.current_state.get_future()
        rclpy.spin_until_future_complete(self.current_state, future)
        self.change_state()

    def change_state(self):
        current_state = type(self.current_state)
        self.current_state.destroy_node()

        if current_state is IdleState:
            if not self.completedFirstRound:
                self.current_state = TakeoffState(self)
            else:
                self.current_state = RegistrationState(self)
        elif current_state is TakeoffState:
            self.current_state = self.registerAfterTakeoff(self)
        elif current_state is ApproachState:
            self.current_state = OrthogonalAlignmentState(self)
        elif current_state is OrthogonalAlignmentState:
            self.current_state = RegistrationState(self)
        elif current_state is RegistrationState:
            if self.completedFirstRound:
                self.current_state = ReturnHomeState(self)
            else:
                self.completedFirstRound = True
                self.current_state = IdleState(self, True)

        self.current_state_publisher.publish(
            String(data=type(self.current_state).__name__))
        if current_state is not ReturnHomeState:
            self.spin_until_state_complete()


def main(args=None):
    rclpy.init(args=args)

    state_machine = WindTurbineInspectionStateMachine()

    rclpy.spin(state_machine)

    state_machine.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
