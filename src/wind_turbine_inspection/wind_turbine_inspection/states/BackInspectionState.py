from wind_turbine_inspection.states.base import InspectionState, WindTurbineInspectionStage
from wind_turbine_inspection.states.constants import windTurbineTypeAndLocation
from std_msgs.msg import String
class BackInspectionState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('back_inspection_state', WindTurbineInspectionStage.BACK_INSPECTION, state_machine)

        self.startInspectionPublisher = self.create_publisher(String, '/drone_control/inspect_wind_turbine', 10)
        self.distanceWaypointPublisher = self.create_publisher(String, '/drone_control/distance_waypoint', 10)
        self.distanceWaypointPublisher.publish(String(data="2,0,0"))
        self.goneForward = False


    def waypoint_reached_callback(self, msg):
        if not self.goneForward:
            self.startInspectionPublisher.publish(String(data=f"{5}"))
            self.goneForward = True
        else:
            self.advance_to_next_state()
