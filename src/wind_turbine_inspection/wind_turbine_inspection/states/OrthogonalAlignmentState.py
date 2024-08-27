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
        self.get_logger().info(f"angle_to_rotate_callback received: {msg.data}")
        if not self.rotating:
            self.rotating = True
            rotateMsg = String()
            rotateMsg.data = f"{msg.data},20"
            self.moveCenteredPublisher.publish(rotateMsg)
        

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"OrthogonalAlignmentState received: {msg.data}")
        # self.advance_to_next_state()
        self.rotating = False
