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

MODE_ALIGNMENT_LIDAR_THRESHOLD = 15
MODE_INSPECTION_LIDAR_THRESHOLD = 2
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
        self.inspection_distances_publisher = self.create_publisher(String, 'inspection_distances', 10)
        self.centroid_distance_was_zero = False

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
        cv2.waitKey(1)
        y_inverted_found, rotorX, rotorY, angle, intersectionsAverageY, vertical_lines = findYShape(copy_img, lines, "Y shape from rgb image")
        avg_dev_with_sign = determine_direction(rotorY, vertical_lines)
        if angle and intersectionsAverageY and avg_dev_with_sign:
            self.angleToHaveWTCenteredOnImagePublisher.publish(String(data=f"{angle},{intersectionsAverageY},{avg_dev_with_sign},0,0"))

    def prepare_image(self, data, threshold_value):
        cv_image = self.br.imgmsg_to_cv2(data, desired_encoding='passthrough')
        cv_image = np.nan_to_num(cv_image, nan=0.0, posinf=0.0, neginf=0.0)
        
        if cv_image is None or cv_image.size == 0 or cv_image.shape[0] == 0 or cv_image.shape[1] == 0:
            return None, None, None, None, None, None
        
        positive_values_mask = cv_image > 0
        positive_values = cv_image[positive_values_mask]
        
        if positive_values.size == 0:
            return None, None, None, None, None, None
        
        min_distance = np.min(positive_values)
        max_distance = min_distance + threshold_value
        
        mask = (cv_image >= min_distance) & (cv_image <= max_distance)
        cv_image_filtered = np.where(mask, cv_image, 0)
        
        # Asegurarse de que max_distance - min_distance no sea cero
        if max_distance - min_distance == 0:
            return None, None, None, None, None, None
        
        cv_normalized = (cv_image_filtered - min_distance) / (max_distance - min_distance)
        cv_normalized = np.clip(cv_normalized, 0, 1)
        cv_8u = (cv_normalized * 255).astype(np.uint8)
        img = cv2.applyColorMap(cv_8u, cv2.COLORMAP_JET)
        
        return cv_image_filtered, positive_values_mask, img, min_distance, cv_normalized, cv_image



    def depth_listener_callback(self, data):
        if self.imageRecognitionState == ImageRecognitionState.OFF:
            return
        
        # Determinar el umbral según el estado
        if self.imageRecognitionState == ImageRecognitionState.ALIGNMENT:
            threshold_value = MODE_ALIGNMENT_LIDAR_THRESHOLD
        elif self.imageRecognitionState == ImageRecognitionState.INSPECTION:
            threshold_value = MODE_INSPECTION_LIDAR_THRESHOLD
        else:
            return
        
        # Preparar la imagen
        cv_image_filtered, positive_values_mask, img, min_distance, cv_normalized, cv_image_original = self.prepare_image(data, threshold_value)
        
        if cv_image_filtered is None:
            return
        
        # Pasar los datos preparados a la función correspondiente
        if self.imageRecognitionState == ImageRecognitionState.ALIGNMENT:
            self.alignment_listener_callback(cv_image_filtered, positive_values_mask, img, min_distance, cv_normalized)
        elif self.imageRecognitionState == ImageRecognitionState.INSPECTION:
            self.inspection_listener_callback(cv_image_filtered, positive_values_mask, img, min_distance, cv_normalized)




    def alignment_listener_callback(self, cv_image_filtered, positive_values_mask, img, min_distance, cv_normalized):
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
            
            y_inverted_found, rotorX, rotorY, angle, percentageRotorY, _ = findYShape(copy_img, lines, "Y shape from depth image")
            if rotorX is not None and rotorY is not None:
                distanceToRotor = get_distance_at_point(rotorX, rotorY, cv_image_filtered)
            else:
                distanceToRotor = None
            if y_inverted_found:
                avg_dev_with_sign = determine_direction_with_depth(y_inverted_found, cv_image_filtered)
            else:
                avg_dev_with_sign = None
            if angle is not None and percentageRotorY is not None and avg_dev_with_sign is not None and distanceToRotor is not None:
                self.angleToHaveWTCenteredOnImagePublisher.publish(String(data=f"{angle},{percentageRotorY},{avg_dev_with_sign},1,{distanceToRotor}"))
        except Exception as e:
            self.get_logger().error(f'Error en alignment_listener_callback: {e}')



    def inspection_listener_callback(self, cv_image_filtered, positive_values_mask, img, min_distance, cv_normalized):
        try:
            # Calculamos el centroide del objeto
            y_coords, x_coords = np.where(positive_values_mask)
            
            if x_coords.size == 0 or y_coords.size == 0:
                return
            
            centroid_x = np.mean(x_coords)
            centroid_y = np.mean(y_coords)
            
            # Dibujar el centroide en la imagen
            cv2.circle(img, (int(centroid_x), int(centroid_y)), 5, (0, 255, 0), -1)
            
            # Mostrar la imagen con el centroide
            cv2.imshow("Depth Image with Centroid", img)
            cv2.waitKey(1)
            
            depthAtCentroid = get_distance_at_point(int(centroid_x), int(centroid_y), cv_image_filtered)
            percentageInXOfCentroid = centroid_x / cv_image_filtered.shape[1]
            percentageInYOfCentroid = centroid_y / cv_image_filtered.shape[0]
            
            if not self.centroid_distance_was_zero and depthAtCentroid == 0:
                self.centroid_distance_was_zero = True
            
            if self.centroid_distance_was_zero and depthAtCentroid > 0:
                self.inspection_distances_publisher.publish(String(data=f"{percentageInXOfCentroid},{percentageInYOfCentroid},{depthAtCentroid}"))
        except Exception as e:
            self.get_logger().error(f'Error en inspection_listener_callback: {e}')
                    
        # copy_img = np.copy(img)

        # lines = preproces_and_hough(img)

        # if lines is None:
        #     return
                    
        # for line in lines:
        #     x1, y1, x2, y2 = line[0]
        #     cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # cv2.imshow('All detected lines', img)
        # cv2.waitKey(1)

        # self.calculate_correction(lines, cv_image, copy_img)

        # except Exception as e:
        #     self.get_logger().error(f'Error en inspection_listener_callback: {e}')


    # # Función para calcular la línea media entre dos líneas
    # def calculate_midline(self, line1, line2):
    #     x11, y11, x12, y12 = line1[0]
    #     x21, y21, x22, y22 = line2[0]
    #     # Calculamos el punto medio entre los extremos de ambas líneas
    #     x1_media = (x11 + x21) // 2
    #     y1_media = (y11 + y21) // 2
    #     x2_media = (x12 + x22) // 2
    #     y2_media = (y12 + y22) // 2
    #     return [[x1_media, y1_media, x2_media, y2_media]]

    # # Devuelve la distancia euclidiana entre dos puntos
    # def distance_between_points(self, p1, p2):
    #     return np.linalg.norm(np.array(p1) - np.array(p2))

    # # Calcula la distancia entre dos líneas
    # def distance_between_lines(self, line1, line2, slope_margin=2):
    #     x1, y1, x2, y2 = line1[0]
    #     x3, y3, x4, y4 = line2[0]

    #     # Calcula las pendientes de ambas líneas
    #     slope1 = slope(line1)
    #     slope2 = slope(line2)

    #     # Verifica si las líneas tienen la misma pendiente
    #     if abs(slope1 - slope2) < slope_margin:
    #         # Si las pendientes son iguales, verifica si están en la misma recta
    #         if self.are_lines_collinear(line1, line2):
    #             return 0  # Las líneas son colineales, por lo tanto la distancia es 0

    #         # Si no son colineales, calcula la distancia perpendicular entre ellas
    #         return self.distance_between_parallel_lines(line1, line2)

    #     # Si no son paralelas, calcula la distancia entre los puntos medios
    #     midpoint1 = self.midpoint((x1, y1), (x2, y2))
    #     midpoint2 = self.midpoint((x3, y3), (x4, y4))
    #     return self.distance_between_points(midpoint1, midpoint2)

    # # Verifica si dos líneas con la misma pendiente son colineales
    # def are_lines_collinear(self, line1, line2, margin=2):
    #     x1, y1, x2, y2 = line1[0]
    #     x3, y3, x4, y4 = line2[0]

    #     # Calcula las pendientes de la línea 1 y entre un punto de la línea 1 y la línea 2
    #     slope_line1 = slope([[x1, y1, x2, y2]])
    #     slope_between_lines = slope([[x1, y1, x3, y3]])

    #     # Si la diferencia entre las pendientes es menor que el margen, se consideran colineales
    #     return abs(slope_line1 - slope_between_lines) <= margin

    # # Calcula la distancia perpendicular entre dos líneas paralelas
    # def distance_between_parallel_lines(self, line1, line2):
    #     # Utiliza la fórmula de distancia entre un punto y una línea
    #     x1, y1, x2, y2 = line1[0]
    #     x3, y3, _, _ = line2[0]

    #     # Ecuación de la línea1: Ax + By + C = 0
    #     A = y2 - y1
    #     B = x1 - x2
    #     C = A * x1 + B * y1

    #     # Calcula la distancia de un punto de la segunda línea a la primera línea
    #     distance = abs(A * x3 + B * y3 - C) / math.sqrt(A**2 + B**2)
    #     return distance

    # def perpendicular_vector(self, x1, y1, x2, y2):
    #     dx = x2 - x1
    #     dy = y2 - y1
    #     return np.array([-dy, dx])

    # # Calcula puntos en la recta perpendicular hatsa que uno tenga distancoa > 0
    # def point_at_distance_from_line(self, point, m, depth_image, max_d=15):
    #     for d in range(1, max_d + 1):
    #         point1 = (0,0)
    #         point2 = (0,0)
    #         if m == float('inf'):  # Línea vertical
    #             point1 = (point[0], point[1] + d)
    #             point2 =(point[0], point[1] - d)
    #         elif m == 0:  # Línea horizontal
    #             point1 = (point[0] + d, point[1])
    #             point2 = (point[0] - d, point[1])
    #         else:
    #             dx = d / np.sqrt(1 + m**2)
    #             dy = m * dx
    #             point1 = (point[0] + dx, point[1] + dy)
    #             point2 = (point[0] - dx, point[1] - dy)

    #         (xp1f, yp1f) = point1
    #         (xp2f, yp2f) = point2
    #         xp1 = int(xp1f)
    #         yp1 = int(yp1f)
    #         xp2 = int(xp2f)
    #         yp2 = int(yp2f)
    #         lidar_value_1 = get_distance_at_point(xp1, yp1, depth_image)
    #         lidar_value_2 = get_distance_at_point(xp2, yp2, depth_image)

    #         if lidar_value_1 > 0:
    #             return (xp1, yp1)
    #         elif lidar_value_2 > 0:
    #             return (xp2, yp2)
    #     return None

    # # Devuelve el punto medio entre dos puntos
    # def midpoint(self, p1, p2):
    #     return (np.array(p1) + np.array(p2)) / 2

    # # Encuentra el punto medio entre dos líneas dadas por sus puntos finales
    # def find_lines_midpoint(self, line1, line2):
    #     x1, y1, x2, y2 = line1[0]
    #     x3, y3, x4, y4 = line2[0]
        
    #     midpoint1 = self.midpoint((x1, y1), (x2, y2))
    #     midpoint2 = self.midpoint((x3, y3), (x4, y4))
        
    #     midpoint_between_lines = self.midpoint(midpoint1, midpoint2)
        
    #     return tuple(midpoint_between_lines)

    # # Encuentra las intersecciones de una línea con los bordes del frame
    # def find_intersection_with_frame(self, m, point, depth_image):
    #     x0, y0 = point
    #     height, width = depth_image.shape
    #     intersections = []

    #     # Intersección con el borde izquierdo (x = 0)
    #     y_at_left = m * (0 - x0) + y0
    #     if 0 <= y_at_left < height:
    #         intersections.append((0, y_at_left))

    #     # Intersección con el borde derecho (x = width - 1)
    #     y_at_right = m * (width - 1 - x0) + y0
    #     if 0 <= y_at_right < height:
    #         intersections.append((width - 1, y_at_right))

    #     # Intersección con el borde superior (y = 0)
    #     if m != 0:  # Evitamos la división por cero para líneas horizontales
    #         x_at_top = (0 - y0) / m + x0
    #         if 0 <= x_at_top < width:
    #             intersections.append((x_at_top, 0))

    #     # Intersección con el borde inferior (y = height - 1)
    #     if m != 0:  # Evitamos la división por cero para líneas horizontales
    #         x_at_bottom = (height - 1 - y0) / m + x0
    #         if 0 <= x_at_bottom < width:
    #             intersections.append((x_at_bottom, height - 1))

    #     return intersections

    # # Determina cuál de las intersecciones está en la dirección correcta según la pendiente y la posición del punto seleccionado
    # def find_correct_intersection(self, selected_point, intersections, m, line_start):
    #     x0, y0 = selected_point
    #     x_start, y_start = line_start
    #     best_intersection = None
    #     max_distance = -float('inf')
        
    #     # Dirección inicial de la línea basada en el punto de partida
    #     direction_x = np.sign(x0 - x_start)  # +1 si la línea se mueve hacia la derecha, -1 si hacia la izquierda
    #     direction_y = np.sign(y0 - y_start)  # +1 si la línea sube, -1 si baja
        
    #     for x, y in intersections:
    #         # Calcular el vector desde el punto seleccionado a la intersección
    #         dx = x - x0
    #         dy = y - y0

    #         # Verificamos que la intersección esté en la dirección correcta en ambos ejes
    #         if (np.sign(dx) == direction_x) and (np.sign(dy) == direction_y):
    #             distance = np.sqrt(dx**2 + dy**2)  # Calculamos la distancia para escoger la más lejana
    #             if distance > max_distance:
    #                 max_distance = distance
    #                 best_intersection = (x, y)

    #     return best_intersection

    # def calculate_correction(self, lines, depth_image, rgb_depth_image, slope_margin=0.1, distance_magin=10):
    #     copy_rgb_depth_img = np.copy(rgb_depth_image)
        
    #     # Almacenar las parejas de líneas que cumplen los criterios
    #     lineas_similares = []

    #     # Comparar todas las combinaciones de líneas
    #     if lines is not None:
    #         num_lines = len(lines)
    #         for i in range(num_lines):
    #             for j in range(i + 1, num_lines):
    #                 line1 = lines[i]
    #                 line2 = lines[j]
                    
    #                 # Calcular las pendientes de las dos líneas
    #                 slope1 = slope(line1)
    #                 slope2 = slope(line2)
                    
    #                 # Verificar si las líneas tienen orientación similar (pendientes similares)
    #                 if abs(slope1 - slope2) < slope_margin:
                            
    #                     # Verificar que las líneas no coincidan y que haya una distancia entre ellas
    #                     distancia = self.distance_between_lines(line1, line2)
    #                     if distancia > distance_magin:
                            
    #                         # Agregar las líneas similares a la lista
    #                         lineas_similares.append((line1, line2))
                            
    #                         # Calcular la línea media entre las dos
    #                         linea_media = self.calculate_midline(line1, line2)
    #                         x1, y1, x2, y2 = linea_media[0]
    #                         cv2.line(rgb_depth_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

    #         # Dibujar las líneas detectadas
    #         for (line1, line2) in lineas_similares:
    #             (xf,yf) = self.find_lines_midpoint(line1, line2)
    #             x = int(xf)
    #             y = int(yf)
    #             cv2.circle(rgb_depth_image, (x, y), 5, (0, 0, 255), -1) # PUNTOS MEDIO ROJO
    #             x11, y11, x12, y12 = line1[0]
    #             x21, y21, x22, y22 = line2[0]
    #             cv2.line(rgb_depth_image, (x11, y11), (x12, y12), (0, 255, 0), 2) # LINEAS VERDES
    #             cv2.line(rgb_depth_image, (x21, y21), (x22, y22), (0, 255, 0), 2)

    #         # Mostrar la imagen con las líneas detectadas y la línea media
    #         cv2.imshow('Lineas Similares y Linea Media', rgb_depth_image)
    #         cv2.waitKey(1)

    #         if len(lineas_similares) == 0: # Solo se ve una linea del aspa
    #             ## aca si o si hay que moverse
    #             line_a = avg_line(lines)
    #             x1, y1, x2, y2 = line_a[0]
    #             # Paso 1: Punto medio de la línea A
    #             midpoint_a = self.midpoint((x1, y1), (x2, y2))

    #             # Paso 2: Pendiente de la línea A y perpendicular
    #             slope_a = slope(line_a)
    #             perpendicular_m = perpendicular_slope(slope_a)

    #             # Paso 3: Punto distancia positiva en la recta B
    #             selected_point = self.point_at_distance_from_line(midpoint_a, perpendicular_m, depth_image)
                
    #             if selected_point is not None:
    #                 cv2.circle(copy_rgb_depth_img, selected_point, 5, (0, 0, 255), -1) # PUNTOS ROJOS

    #                 # Paso 4: Intersección de la recta B con el borde del frame
    #                 intersections = self.find_intersection_with_frame(perpendicular_m, selected_point, depth_image)
    #                 for i in intersections:
    #                     (xif, yif) = i
    #                     xi = int(xif)
    #                     yi = int(yif)
    #                     cv2.circle(copy_rgb_depth_img, (xi, yi), 5, (0, 255, 0), -1) # PUNTO VERDE

    #                 correct_intersection = self.find_correct_intersection(selected_point, intersections, perpendicular_m, midpoint_a)
    #                 if correct_intersection is not None:
    #                     (xcf, ycf) = correct_intersection
    #                     xc = int(xcf)
    #                     yc = int(ycf)
    #                     cv2.circle(copy_rgb_depth_img, (xc, yc), 5, (255, 0, 0), -1) # PUNTO AZUL
    #                     cv2.imshow('Puntos', copy_rgb_depth_img)
    #                     cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
