from wind_turbine_inspection.states.base import InspectionState, WindTurbineInspectionStage
from wind_turbine_inspection.states.constants import windTurbineTypeAndLocation
from std_msgs.msg import String

DESIRED_DISTANCE_TO_WT_ROTOR = 55
DISTANCE_FROM_WAYPOINT_TO_ROTOR = 10

class ApproachState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('approach_state', WindTurbineInspectionStage.APPROACH, state_machine)
        mission_param = self.shared_state['mission_param']
        newCoord = windTurbineTypeAndLocation[mission_param]['coordinates']
        distanceToWaypoint = DESIRED_DISTANCE_TO_WT_ROTOR + DISTANCE_FROM_WAYPOINT_TO_ROTOR
        self.publisher = self.create_publisher(String, '/drone_control/gps_waypoint', 10)
        self.publisher.publish(String(data=f"{newCoord['latitude']},{newCoord['longitude']},{distanceToWaypoint}"))

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"ApproachState received: {msg.data}")
        self.advance_to_next_state()