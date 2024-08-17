import math
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

        # Find configurations of lines that form a 'Y' inverted shape
        y_inverted_found = y_inverted(lines)

        # Draw the detected lines on the image
        if y_inverted_found:
            for line in y_inverted_found:
                x1, y1, x2, y2 = line[0]
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

def slope(line):
    x1, y1, x2, y2 = line[0]
    if x2 == x1:
        return float('inf')
    else:
        return (y2 - y1) / (x2 - x1)

def calculate_angle_between_lines(m1, m2):
    if m1 == float('inf') or m2 == float('inf'):
        # Caso donde una de las líneas es vertical
        if m1 == float('inf') and m2 == float('inf'):
            return 0  # Las dos líneas son paralelas y verticales, ángulo es 0
        elif m1 == float('inf'):
            # m1 es vertical, calcular ángulo con m2
            angle = math.degrees(math.atan(abs(m2)))
            return 90 - angle  # Ángulo con respecto a una línea horizontal
        else:
            # m2 es vertical, calcular ángulo con m1
            angle = math.degrees(math.atan(abs(m1)))
            return 90 - angle  # Ángulo con respecto a una línea horizontal
    else:
        # Caso general donde ninguna línea es vertical
        tan_theta = abs((m2 - m1) / (1 + m1 * m2))
        angle = math.degrees(math.atan(tan_theta))  
        return angle


def are_lines_about_120_degrees(m1, m2, margin_of_error=10):
    # Calcula el ángulo entre las dos líneas
    angle = calculate_angle_between_lines(m1, m2)
    # print('angle', angle)
    
    # Verifica si el ángulo es aproximadamente 120 grados
    return abs(angle - 120) <= margin_of_error or abs(angle - 60) <= margin_of_error

# Devuelve 3 lineas con aprox 120 grados entre ellas
# Se queda con la terna con la linea vertical mas arriba que haya
def y_inverted(lines):
    highest = None
    highest_y = float('inf')

    for comb in combinations(lines, 3):
        line1, line2, line3 = comb

        angle12 = are_lines_about_120_degrees(slope(line1), slope(line2))
        angle23 = are_lines_about_120_degrees(slope(line2), slope(line3))
        angle31 = are_lines_about_120_degrees(slope(line3), slope(line1))

        if angle12 and angle23 and angle31:
            vertical_line = None
            for line in (line1, line2, line3):
                if slope(line) == float('inf'):
                    vertical_line = line
                    break

            if vertical_line is not None:
                x1, y1, x2, y2 = vertical_line[0]
                y_max_vertical = min(y1, y2)
                if y_max_vertical < highest_y:
                    highest = (line1, line2, line3)
                    highest_y = y_max_vertical
    return highest



def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
