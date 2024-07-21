import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory

package_share_directory = get_package_share_directory('wind_turbine_detection')
model_path = os.path.join(package_share_directory, 'resource', 'yolov8n.pt')
model_not_turbine_path = os.path.join(package_share_directory, 'resource', 'yolov8m.pt')

model = YOLO(model_path)
modelNotTurbine = YOLO(model_not_turbine_path)

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera',
            self.listener_callback,
            10)
        self.subscription
        self.br = CvBridge()
        # print(model.names)
        # print(modelNotTurbine.names)
   
    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
        image = current_frame

        # YOLO object detection
        results = model.predict(image)
        img = results[0].plot()
        img2 = np.copy(img)

        # Convert to grayscale
        # cv2.cvtColor: Converts the image to a different color space. Here, it's converting from BGR (Blue, Green, Red) to grayscale.
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cv2.imshow('Grayscale Image', gray)

        # Apply Gaussian Blur
        # blurred = cv2.GaussianBlur(
        #     gray,
        #     (9, 9), # size of the Gaussian kernel
        #     2 # standard deviation in the X direction
        # )
        # cv2.imshow('Blurred Image', blurred)

        # Apply Canny edge detector
        # cv2.Canny: Performs edge detection using the Canny algorithm. The thresholds determine how strong the edges need to be to be detected.
        edges = cv2.Canny(
            gray, 
            10,#50, # lower threshold for the hysteresis procedure 
            150, # upper threshold for the hysteresis procedure 
            apertureSize=3 # size of the Sobel kernel used for finding image gradients. It can be 1, 3, 5, or 7.
        )
        cv2.imshow('Edges', edges)

        # Apply Hough Line Transform
        # cv2.HoughLinesP: Detects lines in the image using the probabilistic Hough Transform. The parameters control the accuracy and characteristics of the detected lines.
        lines = cv2.HoughLinesP(
                    edges,
                    1, # Distance resolution in pixels
                    np.pi/180, # Angle resolution in radians
                    threshold=150, # Min number of votes/intersections for valid line
                    minLineLength=10, # Min allowed length of line
                    maxLineGap=10 # Max allowed gap between points on the same line to link them
                    )

        # Draw lines on the image
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Apply Hough Circle Transform
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT, # The detection method
            dp=1.2, # The inverse ratio of the accumulator resolution to the image resolution
            minDist=50, # The minimum distance between the centers of detected circles
            param1=150, # The higher threshold of the two passed to the Canny edge detector
            param2=30, # The accumulator threshold for the circle centers at the detection stage. Lower values mean more false circles may be detected.
            minRadius=15, # Minimum circle radius
            maxRadius=30 # Maximum circle radius
        )
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                cv2.circle(img, (x, y), r, (0, 0, 255), 2)

        # Display the result
        cv2.imshow('Detected Frame', img)

        lines2 = cv2.HoughLines(
                            edges, 
                            1, # rho: The distance resolution of the accumulator in pixels.
                            np.pi/180, # theta: The angle resolution of the accumulator in radians.
                            150 # threshold: The minimum number of intersections (votes) to detect a line.
        )
 
        # Draw the lines
        if lines2 is not None:
            for rho, theta in lines2[:, 0]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                cv2.line(img2, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.imshow('Detected Frame 2', img2)

        cv2.waitKey(1)
  
def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
