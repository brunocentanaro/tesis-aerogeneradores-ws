import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String
from enum import Enum
from wind_turbine_detection.utils import *

class ImageRecognitionState(Enum):
    OFF = 0
    ALIGNMENT = 1
    INSPECTION = 2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        # self.subscription = self.create_subscription(
        #     Image,
        #     'camera',
        #     self.listener_callback,
        #     10)
        self.depth_subscription = self.create_subscription(
            Image,
            'depth_camera',
            self.depth_listener_callback,
            10)
        self.br = CvBridge()
        self.angleToHaveWTCenteredOnImagePublisher = self.create_publisher(String, 'angle_to_have_wt_centered_on_image', 10)
        self.imageRecognitionState = ImageRecognitionState.OFF
        self.changeModeSubscriber = self.create_subscription(String, 'change_mode', self.change_mode_callback, 10)

    def change_mode_callback(self, msg):
        try:
            self.get_logger().info(f'Cambiando modo a {msg.data}')
            mode = int(msg.data)
            self.imageRecognitionState = ImageRecognitionState(mode)
        except Exception as e:
            self.get_logger().error(f'Error en change_mode_callback: {e}')
   
    def listener_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
        image = current_frame
        img = image
        copy_img = np.copy(img)

        lines = preproces_and_hough(img)

        if lines is None:
            return
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.imshow('All detected lines', img)
        y_inverted_found, rotorX, rotorY, angle, intersectionsAverageY, vertical_lines = findYShape(copy_img, lines, "Y shape from rgb image")
        avg_dev_with_sign = determine_direction(rotorY, vertical_lines)
        if angle and intersectionsAverageY and avg_dev_with_sign:
            self.angleToHaveWTCenteredOnImagePublisher.publish(String(data=f"{angle},{intersectionsAverageY},{avg_dev_with_sign},0"))

    def depth_listener_callback(self, data):
        if self.imageRecognitionState == ImageRecognitionState.OFF:
            return
        if (self.imageRecognitionState == ImageRecognitionState.ALIGNMENT):
            self.alignment_listener_callback(data)
        if (self.imageRecognitionState == ImageRecognitionState.INSPECTION):
            self.inspection_listener_callback(data)


    def alignment_listener_callback(self, data):
        try:
            cv_image = self.br.imgmsg_to_cv2(data, desired_encoding='passthrough')
            cv_image = np.nan_to_num(cv_image, nan=0.0, posinf=0.0, neginf=0.0)

            if (cv_image is None or cv_image.size == 0 or cv_image.shape[0] == 0 or cv_image.shape[1] == 0):
                return
            positive_values = cv_image[cv_image > 0]
            
            if positive_values.size == 0:
                return

            min_distance = np.min(positive_values)
            max_distance = np.max(cv_image)

            cv_normalized = (cv_image - min_distance) / (max_distance - min_distance)
            cv_normalized = np.clip(cv_normalized, 0, 1)

            cv_8u = (cv_normalized * 255).astype(np.uint8)

            img = cv2.applyColorMap(cv_8u, cv2.COLORMAP_JET)
            copy_img = np.copy(img)

            lines = preproces_and_hough(img)

            if lines is None:
                return
                        
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

            cv2.imshow('All detected lines', img)
            
            y_inverted_found, rotorX, rotorY, angle, intersectionsAverageY, vertical_lines = findYShape(copy_img, lines, "Y shape from depth image")

            if rotorX and rotorY:
                distanceToRotor = get_distance_at_point(rotorX, rotorY, cv_image)

                # if distanceToRotor:
                #     self.get_logger().info(f'Distancia en rotor ({rotorX}, {rotorY}): {distanceToRotor} metros')
            
            if y_inverted_found:
                avg_dev_with_sign = determine_direction_with_depth(y_inverted_found, cv_image)                
            
            if angle and intersectionsAverageY and avg_dev_with_sign:
                self.angleToHaveWTCenteredOnImagePublisher.publish(String(data=f"{angle},{intersectionsAverageY},{avg_dev_with_sign},1"))

        except Exception as e:
            self.get_logger().error(f'Error en alignment_listener_callback: {e}')

    def inspection_listener_callback(self, data):
        try:
            cv_image = self.br.imgmsg_to_cv2(data, desired_encoding='passthrough')
            cv_image = np.nan_to_num(cv_image, nan=0.0, posinf=0.0, neginf=0.0)

            if (cv_image is None or cv_image.size == 0 or cv_image.shape[0] == 0 or cv_image.shape[1] == 0):
                return
            positive_values = cv_image[cv_image > 0]
            
            if positive_values.size == 0:
                return

            min_distance = np.min(positive_values)
            max_distance = np.max(cv_image)

            cv_normalized = (cv_image - min_distance) / (max_distance - min_distance)
            cv_normalized = np.clip(cv_normalized, 0, 1)

            cv_8u = (cv_normalized * 255).astype(np.uint8)

            img = cv2.applyColorMap(cv_8u, cv2.COLORMAP_JET)
            copy_img = np.copy(img)

            lines = preproces_and_hough(img)

            if lines is None:
                return
                        
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

            cv2.imshow('All detected lines', img)
            
            # TODO: CONTINUE HERE

        except Exception as e:
            self.get_logger().error(f'Error en inspection_listener_callback: {e}')



def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
