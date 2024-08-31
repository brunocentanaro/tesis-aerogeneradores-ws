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
from std_msgs.msg import String

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
        self.angleToRotatePublisher = self.create_publisher(String, 'angle_to_rotate_centered_on_wt', 10)
        self.angleToHaveWTCenteredOnImagePublisher = self.create_publisher(String, 'angle_to_have_wt_centered_on_image', 10)
        # print(model.names)
        # print(modelNotTurbine.names)
   
    def listener_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
        image = current_frame

        # YOLO object detection
        # results = model.predict(image)
        # img = results[0].plot()
        
        img = image
        img2 = np.copy(img)
        img3 = np.copy(img)

        # Convert to grayscale
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
        edges = cv2.Canny(
            blurred, 
            threshold1=10, # lower threshold for the hysteresis procedure 
            threshold2=150, # upper threshold for the hysteresis procedure 
            apertureSize=3 # size of the Sobel kernel used for finding image gradients. It can be 1, 3, 5, or 7.
        )
        # cv2.imshow('Edges', edges)

        # Apply probabilistic Hough Line Transform
        lines = cv2.HoughLinesP(
            edges,
            rho=1, # Distance resolution in pixels
            theta=np.pi/180, # Angle resolution in radians
            threshold=50, # Min number of votes/intersections for valid line
            minLineLength=40, # Min allowed length of line
            maxLineGap=25 # Max allowed gap between points on the same line to link them
        )

        if lines is None:
            return
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.imshow('All detected lines', img)

        # Find configurations of lines that form a 'Y' inverted shape
        y_inverted_found, verticalLine = y_inverted(lines)

        # Draw the detected lines on the image
        if y_inverted_found:
            intersections = []

            # Encuentra las intersecciones
            for i in range(len(y_inverted_found)):
                for j in range(i + 1, len(y_inverted_found)):
                    intersection = find_line_intersection(y_inverted_found[i], y_inverted_found[j])
                    if intersection:
                        intersections.append(intersection)

            # Dibuja las líneas
            for line in y_inverted_found:
                x1, y1, x2, y2 = line[0]
                cv2.line(img3, (x1, y1), (x2, y2), (0, 255, 0), 2) # LINEAS Y INVERTIDA VERDE

            intersectionsAverageX = 0
            intersectionsAverageY = 0
            for (x, y) in intersections:
                cv2.circle(img3, (x, y), 5, (0, 0, 255), -1) # PUNTOS INTERSECCION ROJO
                intersectionsAverageX += x
                intersectionsAverageY += y

            if intersections:
                intersectionsAverageX = intersectionsAverageX / len(intersections) / img.shape[1]
                intersectionsAverageY = intersectionsAverageY / len(intersections) / img.shape[0]
                cv2.circle(img3, (int(intersectionsAverageX), int(intersectionsAverageY)), 5, (255, 0, 0), -1)
                percentageInImage = (x1 + x2) / 2 / img.shape[1]
                fieldOfView = 1.204 * 180 / math.pi
                self.angleToHaveWTCenteredOnImagePublisher.publish(String(data=f"{percentageInImage * fieldOfView - fieldOfView / 2},{intersectionsAverageY}"))

            cv2.imshow('Y Inverted Shape', img3)

            angle_to_rotate_centered_on_wt = determine_direction(y_inverted_found)
            if angle_to_rotate_centered_on_wt is None:
                return
            self.angleToRotatePublisher.publish(String(data=f"{angle_to_rotate_centered_on_wt}"))

        vertical_lines = []
        horizontal_lines = []
        other_lines = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if is_vertical(x1, y1, x2, y2):
                    vertical_lines.append(line)
                elif is_horizontal(x1, y1, x2, y2):
                    horizontal_lines.append(line)
                else:
                    other_lines.append(line)

        # Dibujar las líneas restantes (ni verticales ni horizontales) en la imagen
        for line in other_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img2, (x1, y1), (x2, y2), (255, 0, 0), 2) # LINEAS RESTANTES EN AZUL
        
        rotor_position = find_rotor_position(other_lines)

        if rotor_position:
            cv2.circle(img2, rotor_position, 5, (0, 0, 255), -1) # POSIBLE ROTOR ROJO
            rotor_x, rotor_y = rotor_position
            for line in vertical_lines:
                x1, y1, x2, y2 = line[0]
                if y1 < rotor_y and y2 < rotor_y:
                    cv2.line(img2, (x1, y1), (x2, y2), (0, 255, 255), 2) # AMARILLO LINEAS ARRIBA DEL ROTOR
                elif y1 > rotor_y and y2 > rotor_y:
                    cv2.line(img2, (x1, y1), (x2, y2), (255, 0, 255), 2) # MAGENTA LINEAS DEBAJO DEL ROTOR
                else:
                    cv2.line(img2, (x1, y1), (x2, y2), (0, 255, 0), 2) # VERDE LINEAS CRUZANDO EL ROTOR
        else:
            print("No possible rotor found")

        cv2.imshow('Remaining Lines', img2)
        
        # Dibujar las líneas verticales en la imagen
        # for line in vertical_lines:
        #     x1, y1, x2, y2 = line[0]
        #     cv2.line(img2, (x1, y1), (x2, y2), (0, 255, 0), 2) # LINEAS VERTICALES VERDE

        # if vertical_lines:
        #     highest = highest_point(vertical_lines)
        #     if highest:
        #         # Dibujar el punto más alto en la imagen
        #         # First estimate of the hub
        #         cv2.circle(img2, highest, 5, (0, 0, 255), -1) # PUNTO MAS ALTO ROJO

        #     distancia_max = 5
        #     possible_blades = close_lines(lines, highest, distancia_max)
        #     # Dibujar posibles aspas
        #     for line in possible_blades:
        #         x1, y1, x2, y2 = line[0]
        #         cv2.line(img2, (x1, y1), (x2, y2), (255, 0, 0), 2) # POSIBLES ASPAS AZUL 

        # cv2.imshow('Line recognition', img2)

        cv2.waitKey(1)

# Busca la intersección más alta de para las lineas que tengan mas de min_angle entre ellas
def find_rotor_position(lines, min_angle=10):
    intersections = []
    for line1, line2 in combinations(lines, 2):
        if calculate_angle_between_lines(slope(line1), slope(line2)) > min_angle:
            intersection = find_line_intersection(line1, line2)

            if intersection:
                intersections.append(intersection)
    
    if intersections:
        rotor_position = min(intersections, key=lambda point: point[1])
        return rotor_position
    return None

# Determina si una línea es vertical dentro de un margen de error
def is_vertical(x1, y1, x2, y2, error_margin=15):
    return abs(x1 - x2) <= error_margin

# Determina si una línea es horizontal dentro de un margen de error
def is_horizontal(x1, y1, x2, y2, error_margin=15):
    return abs(y1 - y2) <= error_margin

# Encuentra el punto más alto en una lista de líneas
def highest_point(lines):
    highest = None
    for line in lines:
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


def are_lines_about_120_degrees(m1, m2, margin_of_error=15):
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
    vertical_line = None

    for comb in combinations(lines, 3):
        line1, line2, line3 = comb

        angle12 = are_lines_about_120_degrees(slope(line1), slope(line2))
        angle23 = are_lines_about_120_degrees(slope(line2), slope(line3))
        angle31 = are_lines_about_120_degrees(slope(line3), slope(line1))

        if angle12 and angle23 and angle31:
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
    return highest, vertical_line

# Encuentra la intersección entre dos líneas representadas por (x1, y1, x2, y2)
def find_line_intersection(line1, line2, tolerance=0.1):
    x1, y1, x2, y2 = line1[0]
    x_1, y_1, x_2, y_2 = line2[0]

    xdiff = (x1 - x2, x_1 - x_2)
    ydiff = (y1 - y2, y_1 - y_2)

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if abs(div) < tolerance:
        return None # Las líneas son casi paralelas o coincidentes

    d = (det((x1, y1), (x2, y2)), det((x_1, y_1), (x_2, y_2)))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div

    return (int(x), int(y))

def determine_direction(lines):
    vertical_edge = None
    left_edge = None
    right_edge = None
    for line in lines:
        m = slope(line)
        if m == float('inf'):
            vertical_edge = line
        elif m  > 0:
            left_edge = line
        else:
            right_edge = line

    if vertical_edge is None or left_edge is None or right_edge is None:
        return None

    vertical_m = slope(vertical_edge)
    left_m = slope(left_edge)
    right_m = slope(right_edge)

    left_angle = calculate_angle_between_lines(left_m, vertical_m)
    right_angle = calculate_angle_between_lines(vertical_m, right_m)
    botton_angle = calculate_angle_between_lines(left_m, right_m)

    if (left_angle < 120):
        return (120 - left_angle) # degree positivo, gira en sentido antihorario
    elif (left_angle > 120):
        return (left_angle - 120) # degree negativo, gira en sentido horario
    else:
        return 0 # no me debo mover

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
