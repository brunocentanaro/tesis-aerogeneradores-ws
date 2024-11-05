from wind_turbine_inspection.states.base import InspectionState, WindTurbineInspectionStage
from wind_turbine_inspection.states.constants import windTurbineTypeAndLocation
from std_msgs.msg import String
import math

CAMERA_FOV = 1.274


class ApproachState(InspectionState):
    def __init__(self, state_machine):
        super().__init__(
            'approach_state',
            WindTurbineInspectionStage.APPROACH,
            state_machine)
        mission_param = self.shared_state['mission_param']
        newCoord = windTurbineTypeAndLocation[mission_param]['coordinates']
        distanceFromGpsToRotor = windTurbineTypeAndLocation[mission_param]['distanceFromGpsToRotor']

        distanceFromVerticalToExtendedBlade = windTurbineTypeAndLocation[mission_param]['bladeLength'] * math.sin(
            math.radians(60))
        distanceToRotorToHaveFullView = distanceFromVerticalToExtendedBlade / \
            math.tan(CAMERA_FOV / 2)

        distanceToHavePartialView = distanceToRotorToHaveFullView * 2 / 3
        distanceToWaypoint = distanceToHavePartialView + distanceFromGpsToRotor
        self.publisher = self.create_publisher(
            String, '/drone_control/gps_waypoint', 10)
        self.publisher.publish(
            String(
                data=f"{newCoord['latitude']},{newCoord['longitude']},{distanceToWaypoint}"))

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"ApproachState received: {msg.data}")
        self.advance_to_next_state()
