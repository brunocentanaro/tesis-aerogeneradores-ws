from wind_turbine_inspection.states.base import InspectionState, WindTurbineInspectionStage
from std_msgs.msg import String

SHOULD_ROTATE_WITHOUT_MOVING_THRESHOLD = 10
MIN_ANGLE_TO_ROTATE = 4
CENTERED_ROTOR_PERCENTAGE_THRESHOLD = 0.07


# TODO: Change this to lidar
VERTICAL_SEEN_DISTANCE = 20
DISTANCE_TO_WIND_TURBINE = 56
class OrthogonalAlignmentState(InspectionState):
    def __init__(self, state_machine):
        super().__init__('orthogonal_alignment_state', WindTurbineInspectionStage.ORTHOGONAL_ALIGNMENT, state_machine)
        self.moveCenteredPublisher = self.create_publisher(String, '/drone_control/rotate_keeping_center', 10)
        
        self.angleToRotateSubscriber = self.create_subscription(String, 'angle_to_rotate_centered_on_wt', self.angle_to_rotate_callback, 10)
 
        self.angleToHaveWTCenteredSubscriber = self.create_subscription(String, 'angle_to_have_wt_centered_on_image', self.angle_to_have_wt_centered_callback, 10)
        self.rotateWithoutMovingPublisher = self.create_publisher(String, '/drone_control/rotate_without_moving', 10)
        self.shouldRotateWithoutMovingCounter = 0
        self.inAnOperation = False
        self.lastFiveWTCenteredAngles = []
        self.lastFivePercentagesInY = []
        self.changeHeightPublisher = self.create_publisher(String, '/drone_control/change_height', 10)
        self.inCorrectPositionCounter = 0
        self.maxInCorrectPositionCounter = 0
        self.todoDeleteAlreadyRotated = False
        self.todoDeleteApproached = False
        self.distanceWaypointPublisher = self.create_publisher(String, '/drone_control/distance_waypoint', 10)
    
    def angle_to_rotate_callback(self, msg):
        data = msg.data.split(',')

        if len(data) == 2:
            avg_dev = float(data[0])
            orientation = data[1]

            self.get_logger().info(f"Received avg_dev: {avg_dev}")
            self.get_logger().info(f"Received orientation: {orientation}")
        else:
            self.get_logger().error("Received data does not match expected format.")
        pass
        # self.get_logger().info(f"angle_to_rotate_callback received: {msg.data}")
        # if not self.rotating:
        #     self.rotating = True
        #     rotateMsg = String()
        #     rotateMsg.data = f"{msg.data},10"
        #     self.moveCenteredPublisher.publish(rotateMsg)
        
    def angle_to_have_wt_centered_callback(self, msg):
        if (self.inAnOperation):
            return
        hadToCorrect = False
        try:
            angle,intersectionYPercentage = map(float, msg.data.split(","))
            lastFiveYPercAverage = sum(self.lastFivePercentagesInY) / len(self.lastFivePercentagesInY) if len(self.lastFivePercentagesInY) > 0 else 0

            if (abs(intersectionYPercentage - lastFiveYPercAverage) < CENTERED_ROTOR_PERCENTAGE_THRESHOLD):
                differenceInY = intersectionYPercentage - 0.5
                if (abs(differenceInY) > 0.05):
                    hadToCorrect = True
                    changeHeightMsg = String()
                    changeHeightMsg.data = f"{(differenceInY)*VERTICAL_SEEN_DISTANCE}"
                    self.changeHeightPublisher.publish(changeHeightMsg)
                    self.inAnOperation = True
                    return
            self.lastFivePercentagesInY.append(intersectionYPercentage)


            
            lastFiveAverage = sum(self.lastFiveWTCenteredAngles) / len(self.lastFiveWTCenteredAngles) if len(self.lastFiveWTCenteredAngles) > 0 else 0
            if (abs(angle - lastFiveAverage) < 5):
                if abs(angle) > MIN_ANGLE_TO_ROTATE:
                    hadToCorrect = True
                    if (self.shouldRotateWithoutMovingCounter < SHOULD_ROTATE_WITHOUT_MOVING_THRESHOLD):
                        self.shouldRotateWithoutMovingCounter += 1
                    else:
                        self.shouldRotateWithoutMovingCounter = 0
                        rotateWithoutMovingMsg = String()
                        rotateWithoutMovingMsg.data = f"{angle}"
                        self.rotateWithoutMovingPublisher.publish(rotateWithoutMovingMsg)
                        self.inAnOperation = True
                else:
                    self.shouldRotateWithoutMovingCounter = 0
            else:
                self.shouldRotateWithoutMovingCounter = 0
            self.lastFiveWTCenteredAngles.append(angle)
            if len(self.lastFiveWTCenteredAngles) > 5:
                self.lastFiveWTCenteredAngles.pop(0)

            if (hadToCorrect):
                if (self.inCorrectPositionCounter > self.maxInCorrectPositionCounter):
                    self.maxInCorrectPositionCounter = self.inCorrectPositionCounter
                    self.get_logger().info(f"Max in correct position counter: {self.maxInCorrectPositionCounter}")
                self.inCorrectPositionCounter = 0
            else:
                self.inCorrectPositionCounter += 1
                if (self.inCorrectPositionCounter > 30):
                    self.inAnOperation = True
                    self.moveCenteredPublisher.publish(String(data=f"13,{DISTANCE_TO_WIND_TURBINE}"))
                    self.todoDeleteAlreadyRotated = True
        except ValueError:
            self.get_logger().error(f"angle_to_have_wt_centered_callback received a non float value: {msg.data}")

    def waypoint_reached_callback(self, msg):
        self.get_logger().info(f"OrthogonalAlignmentState received: {msg.data}")
        # self.advance_to_next_state()

        self.inAnOperation = False
        if (self.todoDeleteAlreadyRotated):
            if (self.todoDeleteApproached):
                self.advance_to_next_state()
            else:
                self.distanceWaypointPublisher.publish(String(data=f"{DISTANCE_TO_WIND_TURBINE-10, 0,0}"))
                self.inAnOperation = True
                self.todoDeleteApproached = True