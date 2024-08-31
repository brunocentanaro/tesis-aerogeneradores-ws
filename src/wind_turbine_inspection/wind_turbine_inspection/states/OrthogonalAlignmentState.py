from wind_turbine_inspection.states.base import InspectionState, WindTurbineInspectionStage
from std_msgs.msg import String

SHOULD_ROTATE_WITHOUT_MOVING_THRESHOLD = 10
MIN_ANGLE_TO_ROTATE = 2
class OrthogonalAlignmentState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('orthogonal_alignment_state', WindTurbineInspectionStage.ORTHOGONAL_ALIGNMENT, state_machine)
        self.moveCenteredPublisher = self.create_publisher(String, '/drone_control/rotate_keeping_center', 10)
        self.moveCenteredPublisher.publish(String(data=""))
        self.angleToRotateSubscriber = self.create_subscription(String, 'angle_to_rotate_centered_on_wt', self.angle_to_rotate_callback, 10)
        self.rotating = False
        self.angleToHaveWTCenteredSubscriber = self.create_subscription(String, 'angle_to_have_wt_centered_on_image', self.angle_to_have_wt_centered_callback, 10)
        self.rotateWithoutMovingPublisher = self.create_publisher(String, '/drone_control/rotate_without_moving', 10)
        self.shouldRotateWithoutMovingCounter = 0
        self.rotatingWithoutMoving = False

    
    def angle_to_rotate_callback(self, msg):
        self.get_logger().info(f"angle_to_rotate_callback received: {msg.data}")
        # if not self.rotating:
        #     self.rotating = True
        #     rotateMsg = String()
        #     rotateMsg.data = f"{msg.data},10"
        #     self.moveCenteredPublisher.publish(rotateMsg)
        
    def angle_to_have_wt_centered_callback(self, msg):
        self.get_logger().info(f"angle_to_have_wt_centered_callback received: {msg.data}")
        if (self.rotatingWithoutMoving):
            return
        try:
            angle = float(msg.data)
            if abs(angle) > MIN_ANGLE_TO_ROTATE:
                if (self.shouldRotateWithoutMovingCounter < SHOULD_ROTATE_WITHOUT_MOVING_THRESHOLD):
                    self.shouldRotateWithoutMovingCounter += 1
                else:
                    self.shouldRotateWithoutMovingCounter = 0
                    rotateWithoutMovingMsg = String()
                    rotateWithoutMovingMsg.data = f"{angle}"
                    self.rotateWithoutMovingPublisher.publish(rotateWithoutMovingMsg)
                    self.rotatingWithoutMoving = True
            else:
                self.shouldRotateWithoutMovingCounter = 0
        except ValueError:
            self.get_logger().error(f"angle_to_have_wt_centered_callback received a non float value: {msg.data}")

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"OrthogonalAlignmentState received: {msg.data}")
        # self.advance_to_next_state()
        self.rotating = False
        self.rotatingWithoutMoving = False