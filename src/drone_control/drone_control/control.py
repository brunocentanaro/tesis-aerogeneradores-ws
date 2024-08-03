import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, ManualControlSetpoint, SensorGps, VehicleGlobalPosition, VehicleLocalPosition
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
        self.current_setpoint = [0.0, 0.0, 0.0]
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

        self.localPositionSubscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.localPositionCallback,
            qos_profile
        )

        self.wayPointsStack = [(0.0, 0.0, -30.0,0.0)]
        self.processing_waypoint = False
        self.localPosition = None

        self.startWindTurbineInspection = self.create_subscription(
            String,
            'inspect_wind_turbine',
            self.inspectWindTurbine,
            10
        )

    def localPositionCallback(self, msg):
        self.localPosition = [msg.x, msg.y, msg.z]
    
    def globalPositionCallback(self, msg):
        self.currentPosition = [msg.lat, msg.lon, msg.alt]

    def listener_callback1(self, msg):
        self.currentCOG = msg.cog_rad

    def gpsWaypointCallback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        try:
            latitude, longitude, altitude = map(float, msg.data.split(','))
            x,y,z,yaw = self.process_new_waypoint(latitude, longitude, altitude)
            self.wayPointsStack.append((x,y,z,yaw))
        except ValueError:
            self.get_logger().error('Invalid waypoint format. Expected format: "latitude,longitude,altitude"')

    def timer_callback(self):
        if self.offboard_setpoint_counter == 200:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.arm()

        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        if self.offboard_setpoint_counter < 201:
            self.offboard_setpoint_counter += 1

        if not self.processing_waypoint and self.wayPointsStack:
            self.processing_waypoint = True
            x,y,z,yaw = self.wayPointsStack.pop(0)
            self.setNewSetpoint(x, y, z, yaw)
            self.get_logger().info('New waypoint set: %s' % str(self.current_setpoint))
            

    def inspectWindTurbine(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        try:
            bladeLength = float(msg.data)
            angleFromHorizontal = 30.0
            x = 0.0
            y = bladeLength * math.cos(math.radians(angleFromHorizontal))
            z = bladeLength * math.sin(math.radians(angleFromHorizontal))
            self.wayPointsStack.append((x,y,z,0.0))
            self.wayPointsStack.append((x, -2 *y, 0, 0.0))
            self.wayPointsStack.append((x,y, -z, 0.0))
            self.wayPointsStack.append((x, 0, -bladeLength, 0.0))
        except ValueError:
            self.get_logger().error('Invalid waypoint format. Expected format: "x,y,z"')

    def distanceWaypointCallback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        try:
            x, y, z = map(float, msg.data.split(','))
            self.setNewSetpoint(x, y, z, self.currentYaw)
        except ValueError:
            self.get_logger().error('Invalid waypoint format. Expected format: "x,y,z"')

    def process_new_waypoint(self, latitude, longitude, altitude):
        if not self.currentPosition or not self.currentCOG:
            self.get_logger().error('No GPS data available')
            self.processing_waypoint = False
            return

        distance, yaw = distance_and_bearing(
            self.currentPosition[0], self.currentPosition[1],
            latitude, longitude
        )
        distanceForward = distance * math.cos(yaw)
        distanceRight = distance * math.sin(yaw)
        return distanceForward, distanceRight, altitude - self.currentPosition[2], yaw

    def setNewSetpoint(self, x, y, z, yaw):
        self.previousYaw = self.currentYaw
        self.previous_setpoint = self.current_setpoint
        self.current_setpoint = [
            self.previous_setpoint[0] + x,
            self.previous_setpoint[1] + y,
            self.previous_setpoint[2] +z
        ]
        self.currentYaw = yaw

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('Disarm command sent')

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self):
        if (self.current_setpoint == [0.0, 0.0, 0.0]):
            return
        msg = TrajectorySetpoint()

        x, y, z = self.current_setpoint
        msg.position = [x, y, z]
        distanceToCoverX = x - self.previous_setpoint[0]
        distanceToCoverY = y - self.previous_setpoint[1]
        distanceToCoverZ = z - self.previous_setpoint[2]
        if self.localPosition is not None:
            distanceToCoverX = x - self.localPosition[0]
            distanceToCoverY = y - self.localPosition[1]
            distanceToCoverZ = z - self.localPosition[2]

        max_distance = max(abs(distanceToCoverX), abs(distanceToCoverY), abs(distanceToCoverZ))
        if max_distance < 0.1:
            self.processing_waypoint = False
        movement_time = max_distance / self.desired_speed
        self.velocity = [
            (distanceToCoverX) / movement_time,
            (distanceToCoverY) / movement_time,
            (distanceToCoverZ) / movement_time
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
