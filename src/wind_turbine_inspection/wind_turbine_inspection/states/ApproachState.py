from wind_turbine_inspection.states.base import InspectionState, WindTurbineInspectionStage
from wind_turbine_inspection.states.constants import windTurbineTypeAndLocation
from std_msgs.msg import String

class ApproachState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('approach_state', WindTurbineInspectionStage.APPROACH, state_machine)
        self.publisher = self.create_publisher(String, '/drone_control/gps_waypoint', 10)
        mission_param = self.shared_state['mission_param']

        self.get_logger().info('Publishing waypoint')
        newCoord = windTurbineTypeAndLocation[mission_param]['coordinates']
        height = windTurbineTypeAndLocation[mission_param]['height']
        self.publish_waypoint(f"{newCoord['latitude']},{newCoord['longitude']},{height}") 

    def publish_waypoint(self, waypoint):
        msg = String()
        msg.data = waypoint
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"ApproachState received: {msg.data}")
        self.advance_to_next_state()