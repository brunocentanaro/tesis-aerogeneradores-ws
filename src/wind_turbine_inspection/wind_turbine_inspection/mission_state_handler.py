import rclpy
from rclpy.node import Node
from wind_turbine_inspection.states.ApproachState import ApproachState
from wind_turbine_inspection.states.RegistrationState import RegistrationState
from wind_turbine_inspection.states.RotationState import RotationState
from wind_turbine_inspection.states.IdleState import IdleState
from wind_turbine_inspection.states.OrthogonalAlignmentState import OrthogonalAlignmentState
from wind_turbine_inspection.states.ReturnHomeState import ReturnHomeState
from wind_turbine_inspection.states.TakeoffState import TakeoffState

class WindTurbineInspectionStateMachine(Node):
    def __init__(self):
        super().__init__('inspection_state_machine')
        self.current_state = IdleState(self)
        self.completedInspectionRounds = 0
        self.spin_until_state_complete()

    def spin_until_state_complete(self):
        future = self.current_state.get_future()
        rclpy.spin_until_future_complete(self.current_state, future)
        self.change_state()

    def change_state(self):
        current_state = type(self.current_state)
        self.current_state.destroy_node()
    
        if current_state is IdleState:
            self.current_state = TakeoffState(self)
        elif current_state is TakeoffState:
            self.current_state = ApproachState(self)
        elif current_state is ApproachState:
            self.current_state = OrthogonalAlignmentState(self)
        elif current_state is OrthogonalAlignmentState:
            self.current_state = RegistrationState(self)
        elif current_state is RegistrationState:
            if self.completedInspectionRounds < 4:
                self.current_state = RotationState(self)
            else:
                self.current_state = ReturnHomeState(self)
        elif current_state is RotationState:
            self.current_state = RegistrationState(self)
        elif current_state is ReturnHomeState:
            self.current_state = IdleState(self)
        else:
            self.current_state = IdleState(self)

        self.spin_until_state_complete()


def main(args=None):
    rclpy.init(args=args)

    state_machine = WindTurbineInspectionStateMachine()

    rclpy.spin(state_machine)

    state_machine.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
