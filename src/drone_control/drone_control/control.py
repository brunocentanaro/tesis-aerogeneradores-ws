import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, ManualControlSetpoint
from std_msgs.msg import String
import time

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.manual_control_setpoint_publisher = self.create_publisher(ManualControlSetpoint, '/fmu/in/manual_control_setpoint', 10)

        self.offboard_setpoint_counter = 0
        self.previous_setpoint = [0.0, 0.0, 0.0]
        self.current_setpoint = [0.0, 0.0, -5.0]  # Inicializar con la posición de despegue
        self.current_manual_control_setpoint = [0.0, 0.0, 0.5, 0.0]

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.subscription = self.create_subscription(
            String,
            'waypoint',
            self.listener_callback,
            10)
        
        self.desired_speed = 0.5
        self.in_manual_speed_mode = True

    def timer_callback(self):
        if self.offboard_setpoint_counter == 200:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.arm()

        self.publish_offboard_control_mode(velocity=self.in_manual_speed_mode, position=not self.in_manual_speed_mode)
        self.publish_trajectory_setpoint()

        if self.offboard_setpoint_counter < 201:
            self.offboard_setpoint_counter += 1

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        try:

            x, y, z = map(float, msg.data.split(','))
            self.previous_setpoint = self.current_setpoint
            self.current_setpoint = [
                self.previous_setpoint[0] + x,
                self.previous_setpoint[1] + y,
                self.previous_setpoint[2] + z
            ]
        except ValueError:
            self.get_logger().error('Invalid waypoint format. Expected format: "x,y,z"')

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('Disarm command sent')

    def publish_offboard_control_mode(self, position=False, velocity=False, acceleration=False, attitude=False, body_rate=False):
        msg = OffboardControlMode()
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = acceleration
        msg.attitude = attitude
        msg.body_rate = body_rate
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.get_logger().info('Publishing offboard control mode' + str(msg))
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()

        msg.position = self.current_setpoint
        x, y, z = self.current_setpoint
        max_distance = max(abs(x), abs(y), abs(z))
        movement_time = max_distance / self.desired_speed
        self.velocity = [
            (x - self.previous_setpoint[0])/ movement_time,
            (y - self.previous_setpoint[1])/ movement_time,
            (z - self.previous_setpoint[2])/ movement_time
        ]
        msg.yaw = 0.0 # [-PI:PI]
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_command_publisher.publish(msg)
    
    def publish_manual_control_setpoint(self):
        msg = ManualControlSetpoint()
        msg.roll = self.current_manual_control_setpoint[0]
        msg.pitch = self.current_manual_control_setpoint[1]
        msg.throttle = self.current_manual_control_setpoint[2]
        msg.yaw = self.current_manual_control_setpoint[3]
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.manual_control_setpoint_publisher.publish(msg)

def main(args=None):
    print("Starting offboard control node...")
    rclpy.init(args=args)
    offboard_control = OffboardControl()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(offboard_control)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
