import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from sympy import Point3D
import math
import cv2
from cv_bridge import CvBridge
import time
from sensor_msgs.msg import Image
import os

DISTANCE_PER_PICTURE = 1
CAMERA_FOV = 1.274
MIN_DISTANCE_TO_TURBINE = 9
HORIZONTAL_PHOTO_DISTANCE_PERFECT = math.sin(
    CAMERA_FOV / 2) * MIN_DISTANCE_TO_TURBINE * 2
DISTANCE_PER_PICTURE = HORIZONTAL_PHOTO_DISTANCE_PERFECT * 0.6

ENABLED = True

class CameraHandler(Node):
    def __init__(self):
        super().__init__('camera_handler')
        if not ENABLED:
            self.get_logger().info('Camera handler disabled')
            return

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile)

        self.subscriber = self.create_subscription(
            String, '/drone_control/waypoint_reached', self.waypoint_reached_callback, 10)
        self.subscription = self.create_subscription(
            Image,
            'camera',
            self.listener_callback,
            10)

        self.shouldTakePictures = False
        self.localPosition = [0, 0, 0]
        self.lastPhotoTakenPosition = [0, 0, 0]
        self.br = CvBridge()
        self.current_image = None

        self.init_timestamp = str(int(time.time()))
        self.images_folder = os.path.join('images', self.init_timestamp)
        os.makedirs(self.images_folder, exist_ok=True)

    def waypoint_reached_callback(self, msg):
        if msg.data == "bladeStart":
            self.shouldTakePictures = True
            self.get_logger().info('Starting to take pictures')
        elif msg.data == "bladeCompleted" or msg.data == "windTurbineCompleted":
            self.shouldTakePictures = False
        elif msg.data == "bladeToBeCompleted":
            self.take_picture()
            self.get_logger().info('Taking picture due to finishing blade')

    def vehicle_local_position_callback(self, msg):
        self.localPosition = [msg.x, msg.y, msg.z]
        if self.shouldTakePictures:
            distance = self.calculate_distance(
                self.localPosition, self.lastPhotoTakenPosition)
            if distance >= DISTANCE_PER_PICTURE:
                self.take_picture()
                self.lastPhotoTakenPosition = self.localPosition

    def listener_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
        self.current_image = current_frame

    def calculate_distance(self, point1, point2):
        p1 = Point3D(point1)
        p2 = Point3D(point2)
        return p1.distance(p2)

    def take_picture(self):
        self.get_logger().info('Taking picture')
        if self.current_image is not None:
            self.lastPhotoTakenPosition = self.localPosition
            timestamp = str(time.time())
            filename = os.path.join(
                self.images_folder,
                f"frame_{timestamp}.png")
            cv2.imwrite(filename, self.current_image)
            print(f"Frame guardado como {filename}")
            self.current_image = None
        else:
            print("No hay ningún frame disponible para guardar.")

def main(args=None):
    rclpy.init(args=args)
    camera_handler = CameraHandler()
    rclpy.spin(camera_handler)
    camera_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
