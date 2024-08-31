from wind_turbine_inspection.states.base import InspectionState, WindTurbineInspectionStage
from std_msgs.msg import String

class OrthogonalAlignmentState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('orthogonal_alignment_state', WindTurbineInspectionStage.ORTHOGONAL_ALIGNMENT, state_machine)
        self.moveCenteredPublisher = self.create_publisher(String, '/drone_control/rotate_keeping_center', 10)
        self.moveCenteredPublisher.publish(String(data=""))
        self.angleToRotateSubscriber = self.create_subscription(String, 'angle_to_rotate', self.angle_to_rotate_callback, 10)
        self.rotating = False

    
    def angle_to_rotate_callback(self, msg):
        data = msg.data.split(',')
        
        if len(data) == 2:
            avg_dev = float(data[0])
            orientation = data[1]
            
            self.get_logger().info(f"Received avg_dev: {avg_dev}")
            self.get_logger().info(f"Received orientation: {orientation}")
        else:
            self.get_logger().error("Received data does not match expected format.")

        # if not self.rotating:
        #     self.rotating = True
        #     rotateMsg = String()
        #     rotateMsg.data = f"{msg.data},10"
        #     self.moveCenteredPublisher.publish(rotateMsg)
        

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"OrthogonalAlignmentState received: {msg.data}")
        # self.advance_to_next_state()
        self.rotating = False
