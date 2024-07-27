import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, ManualControlSetpoint, SensorGps, VehicleGlobalPosition
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import math

def distance_and_bearing(lat1, lon1, lat2, lon2):
    # Convertir grados a radianes
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    
    # Radio de la Tierra en metros
    R = 6371000
    
    # Diferencias de coordenadas
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    
    # Fórmula de Haversine
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    
    # Fórmula para el ángulo
    x = math.sin(dlon) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
    bearing = math.atan2(x, y)
    
    return distance, bearing

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.manual_control_setpoint_publisher = self.create_publisher(ManualControlSetpoint, '/fmu/in/manual_control_setpoint', 10)

        self.offboard_setpoint_counter = 0
        self.previous_setpoint = [0.0, 0.0, 0.0]
        self.current_setpoint = [0.0, 0.0, -5.0]
        self.currentYaw = 0.0
        self.previousYaw = 0.0
        self.current_manual_control_setpoint = [0.0, 0.0, 0.5, 0.0]

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.subscription = self.create_subscription(
            String,
            'distance_waypoint',
            self.distanceWaypointCallback,
            10)
        
        self.gpsWaypointCallback = self.create_subscription(
            String,
            'gps_waypoint',
            self.gpsWaypointCallback,
            10)
        
        self.desired_speed = 0.5
        self.in_manual_speed_mode = True

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=100
        )

        self.subscription = self.create_subscription(
            SensorGps,
            '/fmu/out/vehicle_gps_position',
            self.listener_callback1,
            qos_profile
        )

        self.globalPositionSubscription = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.globalPositionCallback,
            qos_profile
        )
    
    def globalPositionCallback(self, msg):
        self.currentPosition = [msg.lat, msg.lon, msg.alt]


    def listener_callback1(self, msg):
        self.currentCOG = msg.cog_rad

    def gpsWaypointCallback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        try:
            latitude, longitude, altitude = map(float, msg.data.split(','))
            if not self.currentPosition or not self.currentCOG:
                self.get_logger().error('No GPS data available')
                return
        
            distance, yaw = distance_and_bearing(
                self.currentPosition[0], self.currentPosition[1],
                latitude, longitude
            )
            distanceForward = distance * math.cos(yaw)
            distanceRight = distance * math.sin(yaw)
            self.get_logger().info(f'distance: {distance}, yaw: {yaw}')
            self.setNewSetpoint(distanceForward, distanceRight, 0, yaw)
        except ValueError:
            self.get_logger().error('Invalid waypoint format. Expected format: "latitude,longitude,altitude')


    def timer_callback(self):
        if self.offboard_setpoint_counter == 200:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.arm()

        self.publish_offboard_control_mode(velocity=self.in_manual_speed_mode, position=not self.in_manual_speed_mode)
        self.publish_trajectory_setpoint()

        if self.offboard_setpoint_counter < 201:
            self.offboard_setpoint_counter += 1

    def distanceWaypointCallback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        try:

            x, y, z = map(float, msg.data.split(','))
            self.setNewSetpoint(x, y, z, self.currentYaw)
        except ValueError:
            self.get_logger().error('Invalid waypoint format. Expected format: "x,y,z"')

    def setNewSetpoint (self, x, y, z, yaw):
        self.previousYaw = self.currentYaw
        self.previous_setpoint = self.current_setpoint
        self.current_setpoint = [
            self.previous_setpoint[0] + x,
            self.previous_setpoint[1] + y,
            self.previous_setpoint[2] + z,
        ]
        self.currentYaw = yaw

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
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()

        x, y, z = self.current_setpoint
        msg.position = [x,y,z]
        max_distance = max(abs(x), abs(y), abs(z))
        movement_time = max_distance / self.desired_speed
        self.velocity = [
            (x - self.previous_setpoint[0])/ movement_time,
            (y - self.previous_setpoint[1])/ movement_time,
            (z - self.previous_setpoint[2])/ movement_time
        ]
        msg.yaw = self.currentYaw
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
