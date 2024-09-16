from wind_turbine_inspection.states.base import InspectionState, WindTurbineInspectionStage
from wind_turbine_inspection.states.constants import windTurbineTypeAndLocation
from std_msgs.msg import String

class RotationState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('rotation_state', WindTurbineInspectionStage.ROTATION, state_machine)
        self.rotated = False
        if state_machine.completedInspectionRounds == 2:
            windTurbineId = self.shared_state['mission_param']
            self.windTurbineBladeLength = windTurbineTypeAndLocation[windTurbineId]['bladeLength']

            self.distanceWaypointPublisher = self.create_publisher(String, '/drone_control/distance_waypoint', 10)
            self.distanceWaypointPublisher.publish(String(data=f"-{self.windTurbineBladeLength},0,0"))
        else:
            self.rotation_completed()
            self.advance_to_next_state()

    def rotation_completed(self):
        print("Presionar Enter para continuar con la inspeccion...")
        self.rotated = True

    def waypoint_reached_callback(self, msg):
        if not self.rotated:
            self.rotation_completed()
            self.distanceWaypointPublisher.publish(String(data=f"{self.windTurbineBladeLength},0,0"))
        else:
            self.advance_to_next_state()