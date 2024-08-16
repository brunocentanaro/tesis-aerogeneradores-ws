import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory
from itertools import combinations

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
        img3 = np.copy(img)
        img4 = np.copy(img)

        # Convert to grayscale
        # cv2.cvtColor: Converts the image to a different color space. Here, it's converting from BGR (Blue, Green, Red) to grayscale.
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #cv2.imshow('Grayscale Image', gray)

        # Apply Gaussian Blur
        blurred = cv2.GaussianBlur(
            gray,
            (9, 9), # size of the Gaussian kernel
            2 # standard deviation in the X direction
        )
        # cv2.imshow('Blurred Image', blurred)

        # Apply Canny edge detector
        # cv2.Canny: Performs edge detection using the Canny algorithm. The thresholds determine how strong the edges need to be to be detected.
        edges = cv2.Canny(
            blurred, 
            threshold1=10,#50, # lower threshold for the hysteresis procedure 
            threshold2=150, # upper threshold for the hysteresis procedure 
            apertureSize=3 # size of the Sobel kernel used for finding image gradients. It can be 1, 3, 5, or 7.
        )
        # cv2.imshow('Edges', edges)

        # Apply Hough Line Transform
        # cv2.HoughLinesP: Detects lines in the image using the probabilistic Hough Transform. The parameters control the accuracy and characteristics of the detected lines.
        lines = cv2.HoughLinesP(
                    edges,
                    rho=1, # Distance resolution in pixels
                    theta=np.pi/180, # Angle resolution in radians
                    threshold=50, # Min number of votes/intersections for valid line
                    minLineLength=40, # Min allowed length of line
                    maxLineGap=20 # Max allowed gap between points on the same line to link them
                    )

        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.imshow('All detected lines', img)


        # Convert lines to a list of tuples ((x1, y1), (x2, y2))
        converted_lines = [((x1, y1), (x2, y2)) for [[x1, y1, x2, y2]] in lines]

        # Find configurations of lines that form a 'Y' inverted shape
        y_inverted = forms_y_inverted(converted_lines)

        # Draw the detected lines on the image
        if y_inverted:
            for line in y_inverted:
                (x1, y1), (x2, y2) = line
                cv2.line(img4, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.imshow('Y Inverted Shape', img4)
        else:
            print("No 'Y' inverted shape found")


        error_margin = 15  # Ajusta el margen de error según sea necesario
        vertical_lines = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if is_vertical(x1, y1, x2, y2, error_margin):
                    vertical_lines.append(line)

        # Dibujar las líneas verticales en la imagen
        for line in vertical_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img2, (x1, y1), (x2, y2), (0, 255, 0), 2) # LINEAS VERTICALES VERDE

        if vertical_lines:
            highest = highest_point(vertical_lines)
            if highest:
                # Dibujar el punto más alto en la imagen
                # First estimate of the hub
                cv2.circle(img2, highest, 5, (0, 0, 255), -1) # PUNTO MAS ALTO ROJO

            distancia_max = 5
            possible_blades = close_lines(lines, highest, distancia_max)
            # Dibujar posibles aspas
            for line in possible_blades:
                x1, y1, x2, y2 = line[0]
                cv2.line(img2, (x1, y1), (x2, y2), (255, 0, 0), 2) # POSIBLES ASPAS AZUL 

        cv2.imshow('Line recognition', img2)

        # Apply Hough Circle Transform
        # circles = cv2.HoughCircles(
        #     gray,
        #     cv2.HOUGH_GRADIENT, # The detection method
        #     dp=1.2, # The inverse ratio of the accumulator resolution to the image resolution
        #     minDist=50, # The minimum distance between the centers of detected circles
        #     param1=150, # The higher threshold of the two passed to the Canny edge detector
        #     param2=30, # The accumulator threshold for the circle centers at the detection stage. Lower values mean more false circles may be detected.
        #     minRadius=15, # Minimum circle radius
        #     maxRadius=30 # Maximum circle radius
        # )
        # if circles is not None:
        #     circles = np.round(circles[0, :]).astype("int")
        #     for (x, y, r) in circles:
        #         cv2.circle(img, (x, y), r, (0, 0, 255), 2)

        # Display the result

        # Encontrar contornos
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Dibujar todos los contornos
        cv2.drawContours(img3, contours, -1, (0, 255, 0), 2)

        # Mostrar la imagen con contornos
        cv2.imshow('Contours', img3)

        # lines2 = cv2.HoughLines(
        #                     edges, 
        #                     1, # rho: The distance resolution of the accumulator in pixels.
        #                     np.pi/180, # theta: The angle resolution of the accumulator in radians.
        #                     150 # threshold: The minimum number of intersections (votes) to detect a line.
        # )
 
        # Draw the lines
        # if lines2 is not None:
        #     for rho, theta in lines2[:, 0]:
        #         a = np.cos(theta)
        #         b = np.sin(theta)
        #         x0 = a * rho
        #         y0 = b * rho
        #         x1 = int(x0 + 1000 * (-b))
        #         y1 = int(y0 + 1000 * (a))
        #         x2 = int(x0 - 1000 * (-b))
        #         y2 = int(y0 - 1000 * (a))
        #         cv2.line(img2, (x1, y1), (x2, y2), (255, 0, 0), 2)
        # cv2.imshow('Detected Frame 2', img2)

        cv2.waitKey(1)

# Determina si una línea es vertical dentro de un margen de error
def is_vertical(x1, y1, x2, y2, error_margin):
    return abs(x1 - x2) <= error_margin

# Encuentra el punto más alto en una lista de líneas verticales
# Es al reves por el sistema de referencia
def highest_point(vertical_lines):
    highest = None
    for line in vertical_lines:
        x1, y1, x2, y2 = line[0]
        if highest is None or y1 < highest[1]:
            highest = (x1, y1)
        if y2 < highest[1]:
            highest = (x2, y2)
    return highest

# Calcula la distancia euclidiana entre dos puntos p1 y p2
def distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

# Encuentra las líneas que tienen uno de sus extremos a menos de una distancia dada de highest_point
def close_lines(lines, highest_point, distance_max):
    lines_found = []
    
    for line in lines:
        x1, y1, x2, y2 = line[0]
        punto1 = (x1, y1)
        punto2 = (x2, y2)
        
        if distance(punto1, highest_point) <= distance_max or distance(punto2, highest_point) <= distance_max:
            lines_found.append(line)
    
    return lines_found

# Function to calculate the angle of a line in the range [0, 360) degrees
def calculate_angle(x1, y1, x2, y2):
    angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
    return angle % 360  # Convert to [0, 360) range

# Calculate the angle between two lines.
def angle_between_lines(line1, line2):
    angle1 = calculate_angle(*line1[0], *line1[1])
    angle2 = calculate_angle(*line2[0], *line2[1])
    diff = abs(angle1 - angle2) % 360
    return min(diff, 360 - diff)  # Angle between 0 and 180 degrees

# Check if the angle is approximately 120 degrees
def is_approx_120_degrees(angle):
    return np.isclose(angle, 120, atol=45)

# Find configurations of three lines forming a 'Y' inverted shape
def forms_y_inverted(lines):
    for comb in combinations(lines, 3):
        line1, line2, line3 = comb
        angle12 = angle_between_lines(line1, line2)
        angle23 = angle_between_lines(line2, line3)
        angle31 = angle_between_lines(line3, line1)
        
        # print('is_approx_120_degrees 1:', is_approx_120_degrees(angle12))
        # print('is_approx_120_degrees 2:', is_approx_120_degrees(angle23))
        # print('is_approx_120_degrees 3:', is_approx_120_degrees(angle31))

        if is_approx_120_degrees(angle12) and is_approx_120_degrees(angle23) and is_approx_120_degrees(angle31):
            return comb
    return None

# # Function to check if the angle difference is approximately 120 degrees
# def is_approx_120_degrees(angle1, angle2):
#     diff = abs(angle1 - angle2) % 360
#     return np.isclose(diff, 120, atol=10) or np.isclose(diff, 240, atol=10)

# Function to check if three angles form a 120-degree configuration
def forms_120_degrees(angle1, angle2, angle3):
    # Check all possible combinations of differences
    return (
        is_approx_120_degrees(angle1, angle2) and
        is_approx_120_degrees(angle2, angle3) and
        is_approx_120_degrees(angle1, angle3)
    )

# Function to sort lines by angle
def sort_lines_by_angle(lines):
    lines_with_angles = [(line, calculate_angle(line[0][0], line[0][1], line[0][2], line[0][3])) for line in lines]
    lines_with_angles.sort(key=lambda x: x[1])
    sorted_lines = [line for line, angle in lines_with_angles]
    return sorted_lines

# Function to group lines by similar angles
def group_lines_by_angle(lines, angle_threshold):
    sorted_lines = sort_lines_by_angle(lines)
    grouped_lines = []
    current_group = [sorted_lines[0]]

    for i in range(1, len(sorted_lines)):
        _, angle1 = calculate_angle(*sorted_lines[i-1][0])
        _, angle2 = calculate_angle(*sorted_lines[i][0])
        if abs(angle2 - angle1) < angle_threshold:
            current_group.append(sorted_lines[i])
        else:
            grouped_lines.append(current_group)
            current_group = [sorted_lines[i]]

    if current_group:
        grouped_lines.append(current_group)

    return grouped_lines

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
