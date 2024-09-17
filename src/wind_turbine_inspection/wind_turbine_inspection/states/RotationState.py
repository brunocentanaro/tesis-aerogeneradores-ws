from wind_turbine_inspection.states.base import InspectionState, WindTurbineInspectionStage
from wind_turbine_inspection.states.constants import windTurbineTypeAndLocation
from std_msgs.msg import String
from std_srvs.srv import Trigger  # Importamos el servicio Trigger

class RotationState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('rotation_state', WindTurbineInspectionStage.ROTATION, state_machine)
        
        self.srv = self.create_service(Trigger, 'reanudar_inspeccion', self.reanudar_inspeccion_callback)
        if state_machine.completedInspectionRounds == 2:
            self.waiting_command = False
            self.ready_to_advance = False

            windTurbineId = self.shared_state['mission_param']
            self.windTurbineBladeLength = windTurbineTypeAndLocation[windTurbineId]['bladeLength']
            self.distanceWaypointPublisher = self.create_publisher(String, '/drone_control/distance_waypoint', 10)
            self.distanceWaypointPublisher.publish(String(data=f"-{self.windTurbineBladeLength},0,0"))
        else:
            self.waiting_command = True
            self.ready_to_advance = True
    
    def process(self):
        if not self.waiting_command:
            return
        self.waiting_command = False
        if self.ready_to_advance:
            self.advance_to_next_state()
        else:
            self.ready_to_advance = True
            self.distanceWaypointPublisher.publish(String(data=f"{self.windTurbineBladeLength},0,0"))
    
    def reanudar_inspeccion_callback(self, request, response):
        self.process()
        response.success = True
        response.message = "Inspección reanudada"
        return response
    
    def waypoint_reached_callback(self, msg):
        if self.ready_to_advance:
            self.advance_to_next_state()
        else:
            self.waiting_command = True