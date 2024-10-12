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

MODE_ALIGNMENT_LIDAR_THRESHOLD = 30
MODE_INSPECTION_LIDAR_THRESHOLD = 4


class ImageRecognitionState(Enum):
    OFF = 0
    ALIGNMENT = 1
    INSPECTION = 2


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera',
            self.listener_callback,
            10)
        self.depth_subscription = self.create_subscription(
            Image,
            'depth_camera',
            self.depth_listener_callback,
            10)
        self.br = CvBridge()
        self.angleToHaveWTCenteredOnImagePublisher = self.create_publisher(
            String, 'angle_to_have_wt_centered_on_image', 10)
        self.imageRecognitionState = ImageRecognitionState.OFF
        self.changeModeSubscriber = self.create_subscription(
            String, 'change_mode', self.change_mode_callback, 10)
        self.inspection_distances_publisher = self.create_publisher(
            String, 'inspection_distances', 10)
        self.centroid_distance_was_zero = False
        self.imageBr = CvBridge()

    def change_mode_callback(self, msg):
        try:
            self.get_logger().info(f'Cambiando modo a {msg.data}')
            mode = int(msg.data)
            self.imageRecognitionState = ImageRecognitionState(mode)
        except Exception as e:
            self.get_logger().error(f'Error en change_mode_callback: {e}')

    def listener_callback(self, data):
        current_frame = self.imageBr.imgmsg_to_cv2(
            data, desired_encoding="bgr8")
        image = current_frame
        img = image
        copy_img = np.copy(img)

        # lines = preproces_and_hough(img)

        # if lines is None:
        #     return

        # for line in lines:
        #     x1, y1, x2, y2 = line[0]
        #     cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.imshow('All detectednes', img)
        cv2.waitKey(1)
        # y_inverted_found, rotorX, rotorY, angle, intersectionsAverageY, vertical_lines = findYShape(
        #     copy_img, lines, "Y shape from rgb image")
        # avg_dev_with_sign = determine_direction(rotorY, vertical_lines)
        # if angle and intersectionsAverageY and avg_dev_with_sign:
        #     self.angleToHaveWTCenteredOnImagePublisher.publish(
        #         String(data=f"{angle},{intersectionsAverageY},{avg_dev_with_sign},0,0"))

    def prepare_image(self, data, threshold_value):
        cv_image = self.br.imgmsg_to_cv2(data, desired_encoding='passthrough')
        cv_image = np.nan_to_num(cv_image, nan=0.0, posinf=0.0, neginf=0.0)

        if cv_image is None or cv_image.size == 0 or cv_image.shape[
                0] == 0 or cv_image.shape[1] == 0:
            return None, None, None, None, None, None

        positive_values_mask = cv_image > 0
        positive_values = cv_image[positive_values_mask]

        if positive_values.size == 0:
            return None, None, None, None, None, None

        min_distance = np.min(positive_values)
        max_distance = min_distance + threshold_value

        mask = (cv_image >= min_distance) & (cv_image <= max_distance)
        cv_image_filtered = np.where(mask, cv_image, 0)

        if max_distance - min_distance == 0:
            return None, None, None, None, None, None

        cv_normalized = (cv_image_filtered - min_distance) / \
            (max_distance - min_distance)
        cv_normalized = np.clip(cv_normalized, 0, 1)
        cv_8u = (cv_normalized * 255).astype(np.uint8)
        img = cv2.applyColorMap(cv_8u, cv2.COLORMAP_JET)

        return cv_image_filtered, positive_values_mask, img, min_distance, cv_normalized, cv_image

    def depth_listener_callback(self, data):
        if self.imageRecognitionState == ImageRecognitionState.OFF:
            return

        if self.imageRecognitionState == ImageRecognitionState.ALIGNMENT:
            threshold_value = MODE_ALIGNMENT_LIDAR_THRESHOLD
        elif self.imageRecognitionState == ImageRecognitionState.INSPECTION:
            threshold_value = MODE_INSPECTION_LIDAR_THRESHOLD
        else:
            return

        cv_image_filtered, positive_values_mask, img, min_distance, cv_normalized, cv_image_original = self.prepare_image(
            data, threshold_value)

        if cv_image_filtered is None:
            return

        if self.imageRecognitionState == ImageRecognitionState.ALIGNMENT:
            self.alignment_listener_callback(
                cv_image_filtered,
                positive_values_mask,
                img,
                min_distance,
                cv_normalized)
        elif self.imageRecognitionState == ImageRecognitionState.INSPECTION:
            self.inspection_listener_callback(
                cv_image_filtered,
                positive_values_mask,
                img,
                min_distance,
                cv_normalized)

    def alignment_listener_callback(
            self, cv_image_filtered, positive_values_mask, img, min_distance, cv_normalized):
        try:
            copy_img = np.copy(img)
            lines = preproces_and_hough(img)
            if lines is None:
                return

            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

            cv2.imshow('All detected lines', img)
            cv2.waitKey(1)

            y_inverted_found, rotorX, rotorY, angle, percentageRotorY, _ = findYShape(
                copy_img, lines, "Y shape from depth image")
            if rotorX is not None and rotorY is not None:
                distanceToRotor = get_distance_at_point(
                    rotorX, rotorY, cv_image_filtered)
            else:
                distanceToRotor = None
            if y_inverted_found:
                avg_dev_with_sign = determine_direction_with_depth(
                    y_inverted_found, cv_image_filtered)
            else:
                avg_dev_with_sign = None
            if angle is not None and percentageRotorY is not None and avg_dev_with_sign is not None and distanceToRotor is not None:
                self.angleToHaveWTCenteredOnImagePublisher.publish(String(
                    data=f"{angle},{percentageRotorY},{avg_dev_with_sign},1,{distanceToRotor}"))
        except Exception as e:
            self.get_logger().error(
                f'Error en alignment_listener_callback: {e}')

    def inspection_listener_callback(
            self, cv_image_filtered, positive_values_mask, img, min_distance, cv_normalized):
        try:
            indices = np.argwhere(positive_values_mask)

            if indices.size == 0:
                return

            image_center = np.array([
                cv_image_filtered.shape[0] / 2,
                cv_image_filtered.shape[1] / 2
            ])

            distances_to_center = np.linalg.norm(
                indices - image_center, axis=1)
            distances_to_center[distances_to_center == 0] = 1e-6

            weights = 1 / distances_to_center
            normalized_weights = weights / np.sum(weights)

            weighted_sum = np.sum(indices *
                                  normalized_weights[:, np.newaxis], axis=0)
            centroid = weighted_sum
            centroid_y, centroid_x = centroid

            cv2.circle(
                img, (int(
                    round(centroid_x)), int(
                    round(centroid_y))), 5, (0, 255, 0), -1)

            cv2.imshow("Depth Image with Centroid", img)
            cv2.waitKey(1)

            depth_at_centroid = get_distance_at_point(
                int(round(centroid_x)), int(
                    round(centroid_y)), cv_image_filtered
            )

            percentage_in_x_of_centroid = centroid_x / \
                cv_image_filtered.shape[1]
            percentage_in_y_of_centroid = centroid_y / \
                cv_image_filtered.shape[0]

            if not self.centroid_distance_was_zero and depth_at_centroid == 0:
                self.centroid_distance_was_zero = True

            if self.centroid_distance_was_zero and depth_at_centroid > 0:
                self.inspection_distances_publisher.publish(String(
                    data=f"{percentage_in_x_of_centroid},{percentage_in_y_of_centroid},{depth_at_centroid}"
                ))
        except Exception as e:
            self.get_logger().error(
                f'Error en inspection_listener_callback: {e}')


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
