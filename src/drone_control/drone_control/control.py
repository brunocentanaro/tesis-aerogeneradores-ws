import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, ManualControlSetpoint, VehicleLocalPosition, VehicleGlobalPosition, VehicleLocalPosition
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import math
from drone_control.utils import *
from drone_control.path_planner.path_planner import path_planner
from drone_control.path_planner.stl_gen.create_stl import WindTurbine

IN_WAYPOINT_THRESHOLD = 0.2
NEAR_WAYPOINT_THRESHOLD = 0.5
EMPTY_MESSAGE = ""

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        self.previous_setpoint = [0.0, 0.0, 0.0]
        self.current_setpoint = [0.0, 0.0, 0.0]
        self.currentSetpointEndSpeed = [0.0, 0.0, 0.0]
        self.currentYaw = 0.0
        self.previousYaw = 0.0
        self.wayPointsStack = []
        self.processing_waypoint = False
        self.currentLocalPosition = None
        self.nearTicker = 0
        self.maxSpeed = 2.5/9
        self.shouldArmAndTakeoff = False
        self.inTakeoffProcedure = False

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

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=100
        )

        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
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

        self.startWindTurbineInspection = self.create_subscription(
            String,
            'inspect_wind_turbine',
            self.inspectWindTurbine,
            10
        )

        self.advanceToNextWaypoint = self.create_subscription(
            String,
            'advance_to_next_waypoint',
            self.advanceToNextWaypointCallback,
            10
        )

        self.startTakeoffProcedureSubscription = self.create_subscription(
            String,
            'start_takeoff_procedure',
            self.startTakeoffProcedure,
            10
        )
        
        self.moveCenteredSubscription = self.create_subscription(
            String,
            'rotate_keeping_center',
            self.rotateKeepingCenter,
            10
        )

        self.rotateWithoutMovingSubscription = self.create_subscription(
            String,
            'rotate_without_moving',
            self.rotateWithoutMoving,
            10
        )

        self.changeDroneHeight = self.create_subscription(
            String,
            'change_height',
            self.changeDroneHeightCallback,
            10
        )

        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.manual_control_setpoint_publisher = self.create_publisher(ManualControlSetpoint, '/fmu/in/manual_control_setpoint', 10)
        self.waypointReachedPublisher = self.create_publisher(String, 'waypoint_reached', 10)
        self.currentHeading = 0.0
        self.lastInspectionLocation = (0, 0, 0)
        self.wayPointsGroupedForHeading = []


    def changeDroneHeightCallback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        try:
            height = float(msg.data)
            self.wayPointsGroupedForHeading.append([(0.0, 0.0, height, 0.0, 'change height')])

        except ValueError:
            self.get_logger().error('Invalid waypoint format. Expected format: "height"')
            return

    def startTakeoffProcedure(self, msg):
        try:
            takeoffHeight = float(msg.data)
            self.wayPointsGroupedForHeading.append([(0.0, 0.0, -takeoffHeight, 0.0, 'takeoff')])
            self.shouldArmAndTakeoff = True
        except ValueError:
            self.get_logger().error('Invalid waypoint format. Expected format: "height"')
            return

    def advanceToNextWaypointCallback(self, msg):
        self.onWaypointReached()

    def localPositionCallback(self, msg):
        self.currentLocalPosition = [msg.x, msg.y, msg.z]
        
    def globalPositionCallback(self, msg):
        self.currentPosition = [msg.lat, msg.lon, msg.alt]

    def listener_callback1(self, msg):
        self.currentHeading = msg.heading

    def gpsWaypointCallback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        try:
            splitMsg = msg.data.split(',')
            distanceToWaypoint = 0
            if (len(splitMsg) == 2):
                latitude, longitude = map(float, splitMsg)
            if (len(splitMsg) == 3):
                latitude, longitude, distanceToWaypoint = map(float, splitMsg)

            x,y,z,yaw = self.process_new_waypoint(latitude, longitude, distanceToWaypoint)
            self.wayPointsGroupedForHeading.append([(x,y,z,yaw, f"{latitude},{longitude},{z}")])
        except ValueError:
            self.get_logger().error('Invalid waypoint format. Expected format: "latitude,longitude,altitude"')

    def rotateKeepingCenter(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        try:
            newWaypoints = []
            stepsPerDegree = 8

            degrees, distanceToWindTurbine = map(float, msg.data.split(','))
            previousHeading, previousX, previousY = 0, 0, 0
            
            rangeToCover = math.floor(abs(degrees * stepsPerDegree)) 
            direction = 1 if degrees > 0 else -1
            newWaypoints = []

            if rangeToCover == 0:
                self.wayPointsGroupedForHeading.append([(0,0,0,0,'ended rotation')])
                return
            for i in range(rangeToCover):
                x = distanceToWindTurbine - distanceToWindTurbine * math.cos(math.radians(i/ stepsPerDegree))
                y = direction * distanceToWindTurbine * math.sin(math.radians(i / stepsPerDegree))
                newYaw = direction * -math.radians(i / stepsPerDegree) 
                changeX, changeY, changeYaw = x - previousX, y - previousY, newYaw - previousHeading
                

                previousX, previousY, previousHeading = x, y, newYaw

                newWaypoints.append((changeX, changeY, 0, changeYaw, EMPTY_MESSAGE))
            lastWaypoint = newWaypoints[-1]
            newWaypoints[-1] = (lastWaypoint[0], lastWaypoint[1], lastWaypoint[2], lastWaypoint[3], 'centered')
            self.wayPointsGroupedForHeading.append(newWaypoints)
        except ValueError:
            self.get_logger().error('error')


    def rotateWithoutMoving(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        try:
            degrees = float(msg.data)
            stepsPerDegree = 2
            previousYaw = 0
            rangeToCover = math.floor(abs(degrees* stepsPerDegree))
            newWaypoints = []
            if rangeToCover == 0:
                self.wayPointsGroupedForHeading.append([(0,0,0,0,'ended rotation')])
                return
            for i in range(math.floor(abs(degrees)) * stepsPerDegree):
                newYaw = math.radians(i) * (1 if degrees > 0 else -1) / stepsPerDegree
                delta = newYaw - previousYaw
                newWaypoints.append((0, 0, 0, delta, EMPTY_MESSAGE))
                previousYaw = newYaw
            lastSetWaypoint = newWaypoints[-1]
            newWaypoints[-1] = (lastSetWaypoint[0], lastSetWaypoint[1], lastSetWaypoint[2], lastSetWaypoint[3], 'looking to where i need')
            self.wayPointsGroupedForHeading.append(newWaypoints)
        except ValueError:
            self.get_logger().error



    def timer_callback(self):
        if self.shouldArmAndTakeoff:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.arm()
            self.shouldArmAndTakeoff = False

        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        if not self.processing_waypoint:
            if self.wayPointsStack:
                self.processing_waypoint = True
                x, y, z, yaw, message = self.wayPointsStack.pop(0)

                if self.wayPointsStack:
                    xNext, yNext, zNext, _, _ = self.wayPointsStack[0]
                else:
                    xNext, yNext, zNext, yawNext = (0.0, 0.0, 0.0, 0.0)

                maxDiff = max(abs(xNext), abs(yNext), abs(zNext))
                multiplier = self.maxSpeed / maxDiff if maxDiff != 0 else 0.0

                newPossibleSpeed = [xNext * multiplier, yNext * multiplier, zNext * multiplier]

                hasChangeOfDirection = (newPossibleSpeed[0] * self.currentSetpointEndSpeed[0] < 0 or
                    newPossibleSpeed[1] * self.currentSetpointEndSpeed[1] < 0 or
                    newPossibleSpeed[2] * self.currentSetpointEndSpeed[2] < 0)
                
                if hasChangeOfDirection:
                    self.currentSetpointEndSpeed = [0.0, 0.0, 0.0]
                else:
                    self.currentSetpointEndSpeed = newPossibleSpeed

                self.setNewSetpoint(x, y, z, yaw, message)
                return
            elif self.wayPointsGroupedForHeading:
                currentHeading = self.currentHeading
                for waypoint in self.wayPointsGroupedForHeading.pop(0):
                    north = waypoint[0]
                    east = waypoint[1]
                    down = waypoint[2]
                    yawChange = waypoint[3]
                    message = waypoint[4]
                
                    newNorth, newEast, newDown = rotate_ned(north, east, down, currentHeading)
                    self.wayPointsStack.append((newNorth, newEast, newDown, yawChange, message))
    
    def inspectWindTurbine(self, msg):
        rotorDiameter, bladeLength = map(float, msg.split(','))
        self.get_logger().info('Received: "%s"' % msg.data)
        try:
            path = path_planner(WindTurbine(rotorDiameter, bladeLength, "src/drone_control/drone_control/path_planner/stl_gen/turbine"), self.lastInspectionLocation)
            self.get_logger().info('path: %s' % path)
            previous = (0, 0, 0)
            previous_group = None
            newWaypointsGroup = []
            for i in range(len(path)):
                group_id, (x, y, z) = path[i]
                xToUse, yToUse, zToUse = x - previous[0], y - previous[1], z - previous[2]
                if previous_group is not None and group_id == previous_group:
                    newWaypointsGroup.extend(self.addIntermediateWaypoints(xToUse, yToUse, zToUse, 0.0))
                else:
                    newWaypointsGroup.append((xToUse, yToUse, zToUse, 0.0, EMPTY_MESSAGE))
                previous_group = group_id
                previous = (x, y, z)
            _, (x, y, z) = path[-1]
            self.lastInspectionLocation = (x, y, z)
            latestWaypoint = newWaypointsGroup[-1]
            newWaypointsGroup[-1] = (latestWaypoint[0], latestWaypoint[1], latestWaypoint[2], latestWaypoint[3], 'wind turbine')
            self.wayPointsGroupedForHeading.append(newWaypointsGroup)
        except ValueError:
            self.get_logger().error('Invalid waypoint format. Expected format: "x,y,z"')

    def addIntermediateWaypoints(self, x, y, z, yaw):
        maxDistance = max(abs(x), abs(y), abs(z))
        newWaypoints = []
        distancePerWaypoint = 0.3
        numWaypoints = int(maxDistance / distancePerWaypoint)
        xStep = x / numWaypoints
        yStep = y / numWaypoints
        zStep = z / numWaypoints
        yawStep = yaw / numWaypoints
        for i in range(numWaypoints):
            newWaypoints.append((xStep, yStep, zStep, yawStep, EMPTY_MESSAGE))
        return newWaypoints

    def distanceWaypointCallback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        try:
            correctedMsg = msg.data.replace("(", "").replace(")", "")
            self.get_logger().info('Received corrected: "%s"' % correctedMsg)
            x, y, z = map(float, correctedMsg.split(','))
            self.wayPointsGroupedForHeading.append([(x, y, z, 0.0, f"{x},{y},{z}")])
        except ValueError:
            self.get_logger().error('Invalid waypoint format. Expected format: "x,y,z"')

    def process_new_waypoint(self, latitude, longitude, distanceToWaypoint):
        if not self.currentPosition or not self.currentHeading:
            self.get_logger().error('No GPS data available')
            self.processing_waypoint = False
            return

        distance, yaw = getCoordinateInLineToWindTurbineXDistanceBefore(
            self.currentPosition[0], self.currentPosition[1],
            latitude, longitude,
            distanceToWaypoint
        )
        distanceForward = distance * math.cos(yaw)
        distanceRight = distance * math.sin(yaw)
        return distanceForward, distanceRight, 0, yaw

    def setNewSetpoint(self, x, y, z, yaw, message):
        self.previousYaw = self.currentYaw
        self.previous_setpoint = self.current_setpoint
        self.current_setpoint = [
            self.previous_setpoint[0] + x,
            self.previous_setpoint[1] + y,
            self.previous_setpoint[2] + z
        ]
        self.currentYaw = (self.currentYaw + yaw + math.pi) % (2 * math.pi) - math.pi
        self.onWaypointReachedMessage = message

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
        msg.acceleration = True
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
        if self.currentLocalPosition is not None:
            distanceToCoverX = x - self.currentLocalPosition[0]
            distanceToCoverY = y - self.currentLocalPosition[1]
            distanceToCoverZ = z - self.currentLocalPosition[2]

        max_distance = max(abs(distanceToCoverX), abs(distanceToCoverY), abs(distanceToCoverZ))
        yawDistance = abs(self.currentYaw - self.currentHeading) % (2 * math.pi)
        if max_distance < IN_WAYPOINT_THRESHOLD and self.processing_waypoint:
            if (yawDistance < 0.1):
                self.onWaypointReached()
            self.get_logger().info('yawDistance: %s, currentYaw: %s, currentHeading: %s' % (yawDistance, self.currentYaw, self.currentHeading))
        elif max_distance < NEAR_WAYPOINT_THRESHOLD and self.processing_waypoint:
            self.nearTicker += 1
            if self.nearTicker > 50:
                self.get_logger().info('Ticks near this waypoint: %s' % self.nearTicker)
        
        msg.velocity = self.currentSetpointEndSpeed

        msg.yaw = self.currentYaw
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.trajectory_setpoint_publisher.publish(msg)

    def onWaypointReached(self):
            self.processing_waypoint = False
            self.nearTicker = 0
            if (self.onWaypointReachedMessage and self.onWaypointReachedMessage != EMPTY_MESSAGE):
                self.waypointReachedPublisher.publish(String(data=self.onWaypointReachedMessage))
                self.onWaypointReachedMessage = EMPTY_MESSAGE

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


# [mission_state_handler-3] [INFO] [1725919656.798099425] [mission_state_handler]: TakeoffState received: takeoff
# [mission_state_handler-3] [INFO] [1725919656.812242838] [mission_state_handler]: Publishing waypoint
# [mission_state_handler-3] [INFO] [1725919656.813268255] [mission_state_handler]: Publishing: "-34.627221,-54.957851"
# [control-7] [INFO] [1725919656.820182722] [drone_control.control]: Received: "-34.627221,-54.957851"
# [control-7] [INFO] [1725919656.820669800] [drone_control.control]: passing -34.62897298903423, -54.95847999162912, -34.627221, -54.957851, 53
# [control-7] [INFO] [1725919656.821393652] [drone_control.control]: Distance: 149.73267257094273, Yaw: 0.2872517883070938