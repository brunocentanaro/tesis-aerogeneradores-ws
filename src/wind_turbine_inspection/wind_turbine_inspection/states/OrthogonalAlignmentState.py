from wind_turbine_inspection.states.base import InspectionState, WindTurbineInspectionStage
from std_msgs.msg import String
import numpy as np
from enum import Enum

SHOULD_ROTATE_WITHOUT_MOVING_THRESHOLD = 10
MIN_ANGLE_TO_ROTATE = 4
CENTERED_ROTOR_PERCENTAGE_THRESHOLD = 0.1
VERTICAL_SEEN_DISTANCE = 8
APPROACH_STEP = 5
MIN_DISTANCE_FRONT_INSP = 10
MIN_DISTANCE_BACK_INSP = 15
ORTHOGONAL_ERROR_THRESHOLD = 0.5

class AlignmentSubstate(Enum):
    ALIGN_VERTICAL = 1
    ALIGN_HORIZONTAL = 2
    BECOME_ORTHOGONAL = 3
    INTERMEDIATE_APPROACH = 4

class OrthogonalAlignmentState(InspectionState):
    def __init__(self, state_machine):
        super().__init__(
            'orthogonal_alignment_state',
            WindTurbineInspectionStage.ORTHOGONAL_ALIGNMENT,
            state_machine)
        self.moveCenteredPublisher = self.create_publisher(
            String, '/drone_control/rotate_keeping_center', 10)
        self.rotateWithoutMovingPublisher = self.create_publisher(
            String, '/drone_control/rotate_without_moving', 10)
        self.changeHeightPublisher = self.create_publisher(
            String, '/drone_control/change_height', 10)
        self.distanceWaypointPublisher = self.create_publisher(
            String, '/drone_control/distance_waypoint', 10)
        self.angleToHaveWTCenteredSubscriber = self.create_subscription(
            String, 'angle_to_have_wt_centered_on_image', self.angle_to_have_wt_centered_callback, 10)

        self.inAnOperation = False
        self.current_substate = AlignmentSubstate.ALIGN_VERTICAL
        self.lastPercentagesInY = []  # Stores percentages for vertical alignment
        self.lastAngles = []          # Stores angles for horizontal alignment
        self.lastDevs = []            # Stores deviation values for orthogonal alignment
        self.validAlignmentCounter = 0
        self.verticalSeenDistance = VERTICAL_SEEN_DISTANCE
        self.changeImageSubscriberModePublisher = self.create_publisher(
            String, 'change_mode', 10)
        self.changeImageSubscriberModePublisher.publish(String(data="1"))
        # Determine inspection type to set approach distance
        isFrontInspection = self.shared_state['is_front_inspection']
        self.distanceToApproach = MIN_DISTANCE_FRONT_INSP if isFrontInspection else MIN_DISTANCE_BACK_INSP

    def angle_to_have_wt_centered_callback(self, msg):
        if self.inAnOperation:
            return

        try:
            angle, intersectionYPercentage, avgDevWithSign, type, distanceToRotor = map(
                float, msg.data.split(","))

            self.lastPercentagesInY.append(intersectionYPercentage)
            if len(self.lastPercentagesInY) > 20:
                self.lastPercentagesInY.pop(0)

            self.lastAngles.append(angle)
            if len(self.lastAngles) > 20:
                self.lastAngles.pop(0)

            self.lastDevs.append(avgDevWithSign)
            if len(self.lastDevs) > 20:
                self.lastDevs.pop(0)

            if self.current_substate == AlignmentSubstate.ALIGN_VERTICAL:
                # Vertical alignment: adjust height based on median Y percentage
                if len(self.lastPercentagesInY) >= 10:
                    median_y = np.median(self.lastPercentagesInY)
                    differenceInY = median_y - 0.5
                    if abs(differenceInY) > 0.05:
                        # Adjust height
                        changeHeightMsg = String()
                        changeHeightMsg.data = f"{differenceInY * VERTICAL_SEEN_DISTANCE}"
                        self.changeHeightPublisher.publish(changeHeightMsg)
                        self.inAnOperation = True
                        self.get_logger().info(
                            f"Adjusting height by {differenceInY * VERTICAL_SEEN_DISTANCE}")
                    else:
                        # Move to next substate immediately
                        self.current_substate = AlignmentSubstate.ALIGN_HORIZONTAL
                        self.lastAngles = []
                        self.get_logger().info("Vertical alignment complete. Moving to horizontal alignment.")

            elif self.current_substate == AlignmentSubstate.ALIGN_HORIZONTAL:
                # Horizontal alignment: rotate drone (change yaw) based on median angle
                if len(self.lastAngles) >= 10:
                    median_angle = np.median(self.lastAngles)
                    if abs(median_angle) > MIN_ANGLE_TO_ROTATE:
                        # Rotate
                        rotateWithoutMovingMsg = String()
                        rotateWithoutMovingMsg.data = f"{median_angle}"
                        self.rotateWithoutMovingPublisher.publish(
                            rotateWithoutMovingMsg)
                        self.inAnOperation = True
                        self.get_logger().info(f"Rotating by {median_angle}")
                    else:
                        # Move to next substate immediately
                        self.current_substate = AlignmentSubstate.BECOME_ORTHOGONAL
                        self.lastDevs = []
                        self.get_logger().info("Horizontal alignment complete. Moving to become orthogonal.")

            elif self.current_substate == AlignmentSubstate.BECOME_ORTHOGONAL:
                # Orthogonal alignment: adjust positioning to minimize deviation
                if len(self.lastDevs) >= 10:
                    median_dev = np.median(self.lastDevs)
                    errorBetweenBlades = median_dev
                    if abs(errorBetweenBlades) > ORTHOGONAL_ERROR_THRESHOLD:
                        moveCenteredMsg = String()
                        moveCenteredMsg.data = f"{errorBetweenBlades},{distanceToRotor}"
                        self.moveCenteredPublisher.publish(moveCenteredMsg)
                        self.inAnOperation = True
                        self.get_logger().info(
                            f"Adjusting to become orthogonal by {errorBetweenBlades} degrees in {distanceToRotor} m")
                    else:
                        # Move to next substate immediately
                        self.current_substate = AlignmentSubstate.INTERMEDIATE_APPROACH
                        self.get_logger().info("Orthogonal alignment complete. Moving to intermediate approach.")

            elif self.current_substate == AlignmentSubstate.INTERMEDIATE_APPROACH:
                # Gradual approach while ensuring stability
                self.validAlignmentCounter += 1
                if self.validAlignmentCounter >= 20:
                    remaining_distance = distanceToRotor - self.distanceToApproach
                    self.get_logger().info(f"remain: {remaining_distance}")
                    if remaining_distance > 1:
                        approach_distance = min(
                            APPROACH_STEP, remaining_distance)
                        distanceWaypointMsg = String()
                        distanceWaypointMsg.data = f"{approach_distance},0,0"
                        self.distanceWaypointPublisher.publish(
                            distanceWaypointMsg)
                        self.inAnOperation = True
                        self.get_logger().info(
                            f"Approaching ${approach_distance} to distance {distanceToRotor}")
                        self.verticalSeenDistance = max(
                            10, self.verticalSeenDistance - 2)
                        self.current_substate = AlignmentSubstate.ALIGN_VERTICAL
                        self.lastPercentagesInY = []
                        self.lastAngles = []
                        self.lastDevs = []
                        self.validAlignmentCounter = 0
                    else:
                        # Reached minimum distance
                        self.get_logger().info("Reached minimum distance. Inspection complete.")
                        self.advance_to_next_state()
                else:
                    self.get_logger().info(
                        f"Waiting for stability before next approach step ({self.validAlignmentCounter}/20)")

        except ValueError:
            self.get_logger().error(f"Received invalid data: {msg.data}")

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"Waypoint reached: {msg.data}")
        self.inAnOperation = False
        # Clear data arrays when waypoint is reached
        self.lastPercentagesInY.clear()
        self.lastAngles.clear()
        self.lastDevs.clear()
        self.get_logger().info("Data arrays cleared upon reaching waypoint.")

    def advance_to_next_state(self):
        self.changeImageSubscriberModePublisher.publish(String(data="0"))
        super().advance_to_next_state()
