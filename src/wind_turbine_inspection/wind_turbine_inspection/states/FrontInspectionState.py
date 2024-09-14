from wind_turbine_inspection.states.base import InspectionState, WindTurbineInspectionStage
from wind_turbine_inspection.states.constants import windTurbineTypeAndLocation
from std_msgs.msg import String
import sys
#sys.path.append('/home/tesis/tesis-aerogeneradores-ws/src')  # Añadir la ruta al PYTHONPATH

#from path_planner.path_planner import path_planner

class FrontInspectionState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('front_inspection_state', WindTurbineInspectionStage.FRONT_INSPECTION, state_machine)
        windTurbineId = self.shared_state['mission_param']
        windTurbineBladeLength = windTurbineTypeAndLocation[windTurbineId]['bladeLength']

        self.startInspectionPublisher = self.create_publisher(String, '/drone_control/inspect_wind_turbine', 10)
        self.startInspectionPublisher.publish(String(data=f"{windTurbineBladeLength}"))

#    def hola():
#        path_planner()

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"FrontInspectionState received: {msg.data}")
        self.advance_to_next_state()