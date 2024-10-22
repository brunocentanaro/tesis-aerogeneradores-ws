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


class TestingHelper(Node):
    def __init__(self):
        super().__init__('testing_helper')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.startMeasurementState = TakeoffState
        self.endMeasurementState = RegistrationState

        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile)
        self.setpoint_sub = self.create_subscription(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            self.trajectory_setpoint_callback,
            qos_profile)
        self.battery_status_sub = self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status',
            self.battery_status_callback,
            qos_profile)
        self.correction_setpoint_sub = self.create_subscription(
            String,
            '/drone_control/correct_drone_position',
            self.correction_setpoint_callback,
            10)
        self.current_state_sub = self.create_subscription(
            String,
            '/inspection_state_machine/state',
            self.state_callback,
            10)
        self.drone_position_error_sub = self.create_subscription(
            String,
            '/drone_control/position_error',
            self.position_error_callback,
            10)
        self.testResult = None

    def position_error_callback(self, msg):
        if self.testResult is not None and not self.testResult.isCompleted:
            errorNumber = float(msg.data)
            self.testResult.addPositionError(errorNumber)

    def vehicle_local_position_callback(self, msg):
        if self.testResult is not None and not self.testResult.isCompleted:
            self.testResult.addLocalPosition(msg)

    def trajectory_setpoint_callback(self, msg):
        if self.testResult is not None and not self.testResult.isCompleted:
            self.testResult.addSetPoint(msg)

    def battery_status_callback(self, msg):
        pass
        # self.get_logger().info(f"Battery status: {msg}")
        # if self.testResult is not None and not self.testResult.isCompleted:
        #     self.get_logger().info(f"Battery status: {msg}")
        #     self.testResult.addBatteryUsage(msg)

    def correction_setpoint_callback(self, msg):
        if self.testResult is not None and not self.testResult.isCompleted:
            self.testResult.addCorrection(msg.data)

    def state_callback(self, msg):
        self.get_logger().info(f"State: {msg.data}")
        if msg.data == self.startMeasurementState.__name__:
            if self.testResult is None:
                self.get_logger().info("Starting test")
                self.testResult = TestResult()
        elif msg.data == self.endMeasurementState.__name__:
            self.testResult.close(self.get_logger())


def main(args=None):
    rclpy.init(args=args)
    testing_helper = TestingHelper()
    rclpy.spin(testing_helper)
    testing_helper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
