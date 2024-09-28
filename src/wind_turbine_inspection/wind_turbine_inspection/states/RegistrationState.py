from wind_turbine_inspection.states.base import InspectionState, WindTurbineInspectionStage
from wind_turbine_inspection.states.constants import windTurbineTypeAndLocation
from std_msgs.msg import String
import numpy as np

Y_THRESHOLD = 0.3
X_THRESHOLD = 0.1
desired_movement_value_x = 4
desired_movement_value_y = 2.5
desired_movement_value_z = 0.1
MIN_DISTANCE = 8
MAX_DISTANCE = 11
POINTS_NEEDED = 50


class RegistrationState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('registration_state', WindTurbineInspectionStage.INSPECTION, state_machine)
        bladeLength = windTurbineTypeAndLocation[self.shared_state['mission_param']]['bladeLength']
        rotorDiameter = windTurbineTypeAndLocation[self.shared_state['mission_param']]['rotorDiameter']
        self.startInspectionPublisher = self.create_publisher(String, '/drone_control/inspect_wind_turbine', 10)
        self.startInspectionPublisher.publish(String(data=f"{rotorDiameter},{bladeLength}"))
        self.changeImageSubscriberModePublisher = self.create_publisher(String, 'change_mode', 10)
        self.changeImageSubscriberModePublisher.publish(String(data="2"))
        self.inspectionDistancesSubscriber = self.create_subscription(String, 'inspection_distances', self.inspectionDistancesCallback, 10)
        self.correctDronePositionPublisher = self.create_publisher(String, '/drone_control/correct_drone_position', 10)
        self.reEnableProcessingWaypointsPublisher = self.create_publisher(String, '/drone_control/re_enable_processing_waypoints', 10)
        self.inAnOperation = False
        self.lastNeededPercentX = []
        self.lastNeededPercentY = []
        self.lastNeededDistance = []

    def inspectionDistancesCallback(self, msg):
        try:
            percentageInX, percentageInY, distanceAtCentroid = map(float, msg.data.split(","))
            if (self.inAnOperation):
                # self.get_logger().info("En una operación, no se realizará corrección")
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

                stdDevX = np.std(self.lastNeededPercentX)
                stdDevY = np.std(self.lastNeededPercentY)
                stdDevDistance = np.std(self.lastNeededDistance)

                self.get_logger().info(f"x: med: {medianX}, std: {stdDevX}, y: med: {medianY}, std: {stdDevY}, distance: med: {medianDistance}, std: {stdDevDistance}")

                correction_vector = [0, 0, 0, 0]  # [north, east, down, yaw]

                if medianX < X_THRESHOLD:
                    correction_vector[1] = -desired_movement_value_x
                elif medianX > 1 - X_THRESHOLD:
                    correction_vector[1] = desired_movement_value_x

                if medianY < Y_THRESHOLD:
                    correction_vector[2] = -desired_movement_value_y 
                elif medianY > 1 - Y_THRESHOLD:
                    correction_vector[2] = desired_movement_value_y

                if medianDistance < MIN_DISTANCE or medianDistance > MAX_DISTANCE:
                    perfectDistance = (MIN_DISTANCE + MAX_DISTANCE) / 2
                    correction_vector[0] = medianDistance - perfectDistance


                if correction_vector != [0, 0, 0, 0]:
                    corrected_position_msg = f"{correction_vector[0]},{correction_vector[1]},{correction_vector[2]}, {correction_vector[3]}"
                    self.correctDronePositionPublisher.publish(String(data=corrected_position_msg))
                    self.get_logger().info(f"Vector de corrección publicado: {corrected_position_msg}")
                    self.inAnOperation = True
                    self.lastNeededDistance = []
                    self.lastNeededPercentX = []
                    self.lastNeededPercentY = []
                else:
                    self.reEnableProcessingWaypointsPublisher.publish(String(data=""))
        except ValueError:
            self.get_logger().error("Datos recibidos inválidos")
            return

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"RegistrationState received: {msg.data}")
        if msg.data == "correctionSetpoint":
            self.get_logger().info("Waypoint reached: correctionSetpoint")
            self.inAnOperation = False
            return
        self.state_machine.completedInspectionRounds += 1
        self.changeImageSubscriberModePublisher.publish(String(data="0"))
        self.advance_to_next_state()