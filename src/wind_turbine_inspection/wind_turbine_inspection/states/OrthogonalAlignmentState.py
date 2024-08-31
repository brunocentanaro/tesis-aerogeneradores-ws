from wind_turbine_inspection.states.base import InspectionState, WindTurbineInspectionStage
from std_msgs.msg import String

SHOULD_ROTATE_WITHOUT_MOVING_THRESHOLD = 10
MIN_ANGLE_TO_ROTATE = 2

# TODO: Change this to lidar
VERTICAL_SEEN_DISTANCE = 60
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
        self.lastFiveWTCenteredAngles = []
        self.lastFivePercentagesInY = []
        self.changeHeightPublisher = self.create_publisher(String, '/drone_control/change_height', 10)

    
    def angle_to_rotate_callback(self, msg):
        pass
        # self.get_logger().info(f"angle_to_rotate_callback received: {msg.data}")
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
            angle,intersectionYPercentage = map(float, msg.data.split(","))
            lastFiveYPercAverage = sum(self.lastFivePercentagesInY) / len(self.lastFivePercentagesInY) if len(self.lastFivePercentagesInY) > 0 else 0

            if (abs(intersectionYPercentage - lastFiveYPercAverage) < 0.03):
                differenceInY = intersectionYPercentage - 0.5
                if (abs(differenceInY) > 0.03):
                    changeHeightMsg = String()
                    changeHeightMsg.data = f"{(differenceInY)*VERTICAL_SEEN_DISTANCE}"
                    self.changeHeightPublisher.publish(changeHeightMsg)
                    self.rotatingWithoutMoving = True
                    return
            self.lastFivePercentagesInY.append(intersectionYPercentage)


            
            lastFiveAverage = sum(self.lastFiveWTCenteredAngles) / len(self.lastFiveWTCenteredAngles) if len(self.lastFiveWTCenteredAngles) > 0 else 0
            if (abs(angle - lastFiveAverage) < 5):
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
            else:
                self.shouldRotateWithoutMovingCounter = 0
            self.lastFiveWTCenteredAngles.append(angle)
            if len(self.lastFiveWTCenteredAngles) > 5:
                self.lastFiveWTCenteredAngles.pop(0)
        except ValueError:
            self.get_logger().error(f"angle_to_have_wt_centered_callback received a non float value: {msg.data}")

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"OrthogonalAlignmentState received: {msg.data}")
        # self.advance_to_next_state()
        self.rotating = False
        self.rotatingWithoutMoving = False