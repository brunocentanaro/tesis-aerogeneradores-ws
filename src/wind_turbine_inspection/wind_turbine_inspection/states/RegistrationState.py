from wind_turbine_inspection.states.base import InspectionState, WindTurbineInspectionStage
from wind_turbine_inspection.states.constants import windTurbineTypeAndLocation
from std_msgs.msg import String
import numpy as np

Y_THRESHOLD = 0.3
X_THRESHOLD = 0.1
desired_movement_value_x = 4
desired_movement_value_y = 2.5
desired_movement_value_z = 0.1
POINTS_NEEDED = 50
MIN_DISTANCE_FRONT = 8
MAX_DISTANCE_FRONT = 11
MIN_DISTANCE_BACK = 19
MAX_DISTANCE_BACK = 22

class RegistrationState(InspectionState):
    def __init__(self, state_machine):
        super().__init__(
            'registration_state',
            WindTurbineInspectionStage.INSPECTION,
            state_machine)
        bladeLength = windTurbineTypeAndLocation[self.shared_state['mission_param']]['bladeLength']
        rotorDiameter = windTurbineTypeAndLocation[self.shared_state['mission_param']]['rotorDiameter']
        self.startInspectionPublisher = self.create_publisher(
            String, '/drone_control/inspect_wind_turbine', 10)
        self.startInspectionPublisher.publish(
            String(data=f"{rotorDiameter},{bladeLength}"))
        self.changeImageSubscriberModePublisher = self.create_publisher(
            String, 'change_mode', 10)

        imageSubscriberMode = "2" if self.shared_state['is_front_inspection'] else "3"
        self.changeImageSubscriberModePublisher.publish(
            String(data=imageSubscriberMode))

        self.inspectionDistancesSubscriber = self.create_subscription(
            String, 'inspection_distances', self.inspectionDistancesCallback, 10)
        self.correctDronePositionPublisher = self.create_publisher(
            String, '/drone_control/correct_drone_position', 10)
        self.reEnableProcessingWaypointsPublisher = self.create_publisher(
            String, '/drone_control/re_enable_processing_waypoints', 10)
        self.inAnOperation = False
        self.lastNeededPercentX = []
        self.lastNeededPercentY = []
        self.lastNeededDistance = []
        self.bladeCompletedCounter = 0
        self.MIN_DISTANCE = MIN_DISTANCE_FRONT if self.shared_state[
            'is_front_inspection'] else MIN_DISTANCE_BACK
        self.MAX_DISTANCE = MAX_DISTANCE_FRONT if self.shared_state[
            'is_front_inspection'] else MAX_DISTANCE_BACK

    def inspectionDistancesCallback(self, msg):
        try:
            percentageInX, percentageInY, distanceAtCentroid = map(
                float, msg.data.split(","))
            if (self.inAnOperation):
                # self.get_logger().info("In an operation, no correction will be made")
                return

            self.lastNeededPercentX.append(percentageInX)
            self.lastNeededPercentY.append(percentageInY)
            self.lastNeededDistance.append(distanceAtCentroid)
            if len(self.lastNeededPercentX) > POINTS_NEEDED:
                self.lastNeededPercentX.pop(0)
            if len(self.lastNeededPercentY) > POINTS_NEEDED:
                self.lastNeededPercentY.pop(0)
            if len(self.lastNeededDistance) > POINTS_NEEDED:
                self.lastNeededDistance.pop(0)

            if len(self.lastNeededPercentX) == POINTS_NEEDED:
                medianX = np.median(self.lastNeededPercentX)
                medianY = np.median(self.lastNeededPercentY)
                medianDistance = np.median(self.lastNeededDistance)
                correction_vector = [0, 0, 0, 0]  # [north, east, down, yaw]

                # Compute corrections based on thresholds
                if medianX < X_THRESHOLD:
                    correction_vector[1] = -desired_movement_value_x
                elif medianX > 1 - X_THRESHOLD:
                    correction_vector[1] = desired_movement_value_x

                if medianY < Y_THRESHOLD:
                    correction_vector[2] = -desired_movement_value_y
                elif medianY > 1 - Y_THRESHOLD:
                    correction_vector[2] = desired_movement_value_y

                if medianDistance < self.MIN_DISTANCE or medianDistance > self.MAX_DISTANCE:
                    perfectDistance = (
                        self.MIN_DISTANCE + self.MAX_DISTANCE) / 2
                    correction_vector[0] = medianDistance - perfectDistance

                # Publish correction if needed
                if correction_vector != [0, 0, 0, 0]:
                    corrected_position_msg = f"{correction_vector[0]},{correction_vector[1]},{correction_vector[2]}, {correction_vector[3]}"
                    self.correctDronePositionPublisher.publish(
                        String(data=corrected_position_msg))
                    self.get_logger().info(
                        f"Correction vector published: {corrected_position_msg}")
                    self.inAnOperation = True
                    self.lastNeededDistance = []
                    self.lastNeededPercentX = []
                    self.lastNeededPercentY = []
                else:
                    self.reEnableProcessingWaypointsPublisher.publish(
                        String(data=""))
        except ValueError:
            self.get_logger().error("Invalid data received")
            return

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"RegistrationState received: {msg.data}")
        if msg.data == "correctionSetpoint":
            self.get_logger().info("Waypoint reached: correctionSetpoint")
            self.inAnOperation = False
            return
        elif msg.data == "bladeCompleted" and self.bladeCompletedCounter < 2:
            self.lastNeededDistance = []
            self.lastNeededPercentX = []
            self.lastNeededPercentY = []
            self.inAnOperation = True
            self.get_logger().info("Waypoint reached: bladeCompleted")
            self.bladeCompletedCounter += 1
            return
        elif msg.data == "bladeStart":
            self.get_logger().info("Waypoint reached: bladeStart")
            self.inAnOperation = False
            return

        if (msg.data != "windTurbineCompleted"):
            return
        self.reEnableProcessingWaypointsPublisher.publish(
            String(data=""))
        self.changeImageSubscriberModePublisher.publish(String(data="0"))
        self.advance_to_next_state()
