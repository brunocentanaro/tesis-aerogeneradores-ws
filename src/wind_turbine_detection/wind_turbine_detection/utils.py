import math
import cv2
import numpy as np
from itertools import combinations
from wind_turbine_detection.constants import CAMERA_FOV

def preproces_and_hough(image):
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian Blur
    blurred = cv2.GaussianBlur(
        gray,
        (9, 9), # size of the Gaussian kernel
        2 # standard deviation in the X direction
    )

    # Apply Canny edge detector
    edges = cv2.Canny(
        blurred, 
        threshold1=10, # lower threshold for the hysteresis procedure 
        threshold2=150, # upper threshold for the hysteresis procedure 
        apertureSize=3 # size of the Sobel kernel used for finding image gradients. It can be 1, 3, 5, or 7.
    )

    # Apply probabilistic Hough Line Transform
    lines = cv2.HoughLinesP(
        edges,
        rho=1, # Distance resolution in pixels
        theta=np.pi/180, # Angle resolution in radians
        threshold=50, # Min number of votes/intersections for valid line
        minLineLength=40, # Min allowed length of line
        maxLineGap=25 # Max allowed gap between points on the same line to link them
    )
    return lines

# Función para calcular la línea media entre dos líneas
def calculate_midline(line1, line2):
    x11, y11, x12, y12 = line1[0]
    x21, y21, x22, y22 = line2[0]
    # Calculamos el punto medio entre los extremos de ambas líneas
    x1_media = (x11 + x21) // 2
    y1_media = (y11 + y21) // 2
    x2_media = (x12 + x22) // 2
    y2_media = (y12 + y22) // 2
    return (x1_media, y1_media, x2_media, y2_media)

def calculate_correction(lines, depth_image, slope_margin=0.1, distance_magin=10):
    # Almacenar las parejas de líneas que cumplen los criterios
    lineas_similares = []

    # Comparar todas las combinaciones de líneas
    if lines is not None:
        num_lines = len(lines)
        for i in range(num_lines):
            for j in range(i + 1, num_lines):
                line1 = lines[i][0]
                line2 = lines[j][0]
                
            # Calcular las pendientes de las dos líneas
            slope1 = slope(line1)
            slope2 = slope(line2)
            
            # Verificar si las líneas tienen orientación similar (pendientes similares)
            if abs(slope1 - slope2) < slope_margin:
                    
                # Verificar que las líneas no coincidan y que haya una distancia entre ellas
                distancia = distance_between_lines(line1, line2)
                if distancia > distance_magin:
                    
                    # Agregar las líneas similares a la lista
                    lineas_similares.append((line1, line2))
                    
                    # Calcular la línea media entre las dos
                    linea_media = calculate_midline(line1, line2)
                    x1, y1, x2, y2 = linea_media[0]
                    cv2.line(depth_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

        # Dibujar las líneas detectadas
        print(f"lineas_similares: {lineas_similares}")
        for (line1, line2) in lineas_similares:
            (x,y) = find_lines_midpoint(line1, line2)
            cv2.circle(depth_image, (x, y), 5, (0, 0, 255), -1) # PUNTOS MEDIO ROJO
            x11, y11, x12, y12 = line1[0]
            x21, y21, x22, y22 = line2[0]
            cv2.line(depth_image, (x11, y11), (x12, y12), (0, 255, 0), 2) # LINEAS VERDES
            cv2.line(depth_image, (x21, y21), (x22, y22), (0, 255, 0), 2)

        # Mostrar la imagen con las líneas detectadas y la línea media
        cv2.imshow('Lineas Similares y Linea Media', depth_image)

        if len(lineas_similares) == 0: # Solo se ve una linea del aspa
            ## aca si o si hay que moverse
            distance = 5
            line_a = avg_line(lines)
            x1, y1, x2, y2 = line_a[0]
            # Paso 1: Punto medio de la línea A
            midpoint_a = midpoint((x1, y1), (x2, y2))

            # Paso 2: Pendiente de la línea A y perpendicular
            slope_a = slope((x1, y1), (x2, y2))
            perpendicular_m = perpendicular_slope(slope_a)

            # Paso 3: Puntos a una distancia paramétrica en la recta B
            p1, p2 = points_at_distance_from_line(midpoint_a, perpendicular_m)
            (xp1, yp1) = p1
            (xp2, yp2) = p2
            cv2.circle(depth_image, (xp1, yp1), 5, (0, 0, 255), -1) # PUNTOS ROJOS
            cv2.circle(depth_image, (xp2, yp2), 5, (0, 0, 255), -1) # PUNTOS ROJOS

            # Paso 4: Valores del LIDAR en esos puntos
            lidar_value_1 = get_distance_at_point(xp1, yp1, depth_image)
            lidar_value_2 = get_distance_at_point(xp2, yp2, depth_image)

            # Determina el punto con mayor valor del LIDAR
            if lidar_value_1 > lidar_value_2:
                selected_point = p1
            else:
                selected_point = p2
            cv2.circle(depth_image, selected_point, 5, (255, 0, 0), -1) # PUNTO AZUL

            # Paso 5: Intersección de la recta B con el borde del frame
            intersections = find_intersection_with_frame(perpendicular_m, selected_point, depth_image)
            for i in intersections:
                cv2.circle(depth_image, i, 5, (0, 255, 0), -1) # PUNTO VERDE

            correct_intersection = find_correct_intersection(selected_point, intersections, perpendicular_m, midpoint_a)
            cv2.circle(depth_image, correct_intersection, 5, (255, 0, 0), -1) # PUNTO AZUL
            cv2.imshow('Puntos', depth_image)


# Devuelve la distancia euclidiana entre dos puntos
def distance_between_points(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

# Calcula la distancia entre dos líneas basándose en la distancia entre sus puntos medios
def distance_between_lines(line1, line2):
    x1, y1, x2, y2 = line1[0]
    x3, y3, x4, y4 = line2[0]
    
    # Encuentra los puntos medios de cada línea
    midpoint1 = midpoint((x1, y1), (x2, y2))
    midpoint2 = midpoint((x3, y3), (x4, y4))
    
    # Calcula la distancia entre los puntos medios
    distance = distance_between_points(midpoint1, midpoint2)
    
    return distance

def perpendicular_vector(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    return np.array([-dy, dx])

# Calcula dos puntos en la recta perpendicular a una distancia d
def points_at_distance_from_line(point, m, d=5):
    if m == float('inf'):  # Línea vertical
        return (point[0], point[1] + d), (point[0], point[1] - d)
    elif m == 0:  # Línea horizontal
        return (point[0] + d, point[1]), (point[0] - d, point[1])
    else:
        dx = d / np.sqrt(1 + m**2)
        dy = m * dx
        return (point[0] + dx, point[1] + dy), (point[0] - dx, point[1] - dy)

# Devuelve el punto medio entre dos puntos
def midpoint(p1, p2):
    return (np.array(p1) + np.array(p2)) / 2

# Encuentra el punto medio entre dos líneas dadas por sus puntos finales
def find_lines_midpoint(line1, line2):
    x1, y1, x2, y2 = line1[0]
    x3, y3, x4, y4 = line2[0]
    
    midpoint1 = midpoint((x1, y1), (x2, y2))
    midpoint2 = midpoint((x3, y3), (x4, y4))
    
    midpoint_between_lines = midpoint(midpoint1, midpoint2)
    
    return tuple(midpoint_between_lines)

# Encuentra las intersecciones de una línea con los bordes del frame
def find_intersection_with_frame(m, point, depth_image):
    x0, y0 = point
    height, width = depth_image.shape
    intersections = []

    # Intersección con el borde izquierdo (x = 0)
    y_at_left = m * (0 - x0) + y0
    if 0 <= y_at_left < height:
        intersections.append((0, y_at_left))

    # Intersección con el borde derecho (x = width - 1)
    y_at_right = m * (width - 1 - x0) + y0
    if 0 <= y_at_right < height:
        intersections.append((width - 1, y_at_right))

    # Intersección con el borde superior (y = 0)
    if m != 0:  # Evitamos la división por cero para líneas horizontales
        x_at_top = (0 - y0) / m + x0
        if 0 <= x_at_top < width:
            intersections.append((x_at_top, 0))

    # Intersección con el borde inferior (y = height - 1)
    if m != 0:  # Evitamos la división por cero para líneas horizontales
        x_at_bottom = (height - 1 - y0) / m + x0
        if 0 <= x_at_bottom < width:
            intersections.append((x_at_bottom, height - 1))

    return intersections

def find_correct_intersection(selected_point, intersections, m, line_start):
    """Determina cuál de las intersecciones está en la dirección correcta según la pendiente y la posición del punto seleccionado.

    Args:
        selected_point: El punto de partida de la recta B.
        intersections: Lista de intersecciones calculadas.
        m: Pendiente de la recta B.
        line_start: Punto de inicio de la línea B (define la dirección inicial de la línea).

    Returns:
        La intersección que sigue la dirección de la recta B.
    """
    x0, y0 = selected_point
    x_start, y_start = line_start
    best_intersection = None
    max_distance = -float('inf')
    
    # Dirección inicial de la línea basada en el punto de partida
    direction_x = np.sign(x0 - x_start)  # +1 si la línea se mueve hacia la derecha, -1 si hacia la izquierda
    direction_y = np.sign(y0 - y_start)  # +1 si la línea sube, -1 si baja
    
    for x, y in intersections:
        # Calcular el vector desde el punto seleccionado a la intersección
        dx = x - x0
        dy = y - y0

        # Verificamos que la intersección esté en la dirección correcta en ambos ejes
        if (np.sign(dx) == direction_x) and (np.sign(dy) == direction_y):
            distance = np.sqrt(dx**2 + dy**2)  # Calculamos la distancia para escoger la más lejana
            if distance > max_distance:
                max_distance = distance
                best_intersection = (x, y)

    return best_intersection

def determine_direction_with_depth(y_inverted_found, depth_image):
    vertical_edge = None
    left_edge = None
    right_edge = None

    for line in y_inverted_found:
        m = slope(line)
        if m == float('inf'):
            vertical_edge = line
        elif m < 0:
            left_edge = line
        else:
            right_edge = line

    if vertical_edge is None or left_edge is None or right_edge is None:
        return None

    lx1, ly1, lx2, ly2 = left_edge[0]
    rx1, ry1, rx2, ry2 = right_edge[0]

    upper_ly = min(ly1, ly2)
    lower_ly = max(ly1, ly2)

    upper_ry = min(ry1, ry2)
    lower_ry = max(ry1, ry2)

    lower_y_coincidence = None
    if lower_ly <= lower_ry and lower_ly >= upper_ry:
        lower_y_coincidence = lower_ly
    elif lower_ry <= lower_ly and lower_ry >= upper_ly:
        lower_y_coincidence = lower_ry
    
    if lower_y_coincidence:        
        # Calcula la pendiente (m) e intersección (b) de ambas líneas
        m_left = slope(left_edge)
        b_left = intercept(left_edge, m_left)
        
        m_right = slope(right_edge)
        b_right = intercept(right_edge, m_right)
        
        # Obtén el valor de x para el y de lower_coincidence en ambas líneas
        x_left = x_at_y(m_left, b_left, lower_y_coincidence, lx1 if m_left == float('inf') else None)
        x_right = x_at_y(m_right, b_right, lower_y_coincidence, rx1 if m_right == float('inf') else None)
        
        distance_left = get_distance_at_approx_x_point(x_left, lower_y_coincidence, depth_image)
        distance_right = get_distance_at_approx_x_point(x_right, lower_y_coincidence, depth_image)

        if distance_left is None or distance_right is None:
            return None

        avg_dev = (abs(distance_left - distance_right))

        orientation = 0
        if distance_left < distance_right:
            orientation = 1 # Sentido antihorario: 1
        elif distance_left > distance_right:
            orientation = -1 # Sentido horario: -1

        return (avg_dev * orientation)
    return None

# Calcula la intersección con el eje y (b) de la ecuación y = mx + b
def intercept(line, m):
    if m == float('inf'):
        return None  # Línea vertical, no tiene intersección en el eje y
    x1, y1, _, _ = line[0]
    return y1 - m * x1

# Calcula el valor de x dado un valor de y en la ecuación y = mx + b
# o devuelve el valor constante de x si la línea es vertical
def x_at_y(m, b, y, vertical_x=None):
    if m == float('inf'):
        return vertical_x  # Línea vertical, siempre tiene un x constante
    return (y - b) / m

def get_distance_at_approx_x_point(x, y, depth_image, max_margin=5):
    distance = get_distance_at_point(x, y, depth_image)
    if distance > 0:
        return distance
    
    for margin in range(1, max_margin + 1):
        # Verifica en las posiciones a la izquierda y a la derecha del punto (x, y)
        for dx in [margin, -margin]:
            distance = get_distance_at_point(x + dx, y, depth_image)
            if distance > 0:
                return distance
            
        # # Verifica en las posiciones adicionales alrededor del punto (x, y)
        # for dx in range(1, margin):
        #     for dy in [margin, -margin]:
        #         distance = get_distance_at_point(x + dx, y + dy, depth_image)
        #         if distance > 0:
        #             return distance
                
        #         distance = get_distance_at_point(x - dx, y + dy, depth_image)
        #         if distance > 0:
        #             return distance

    return None

def get_distance_at_point(x, y, depth_image):
    x = math.floor(x)
    y = math.floor(y)
    # Verifica si el punto está dentro de los límites de la imagen
    if 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
        # Obtiene el valor de distancia en el punto (x, y)
        distance = depth_image[y, x]
        return distance

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

# Calcula la pendiente de la línea 
def slope(line):
    x1, y1, x2, y2 = line[0]
    if x2 == x1:
        return float('inf')
    else:
        return (y2 - y1) / (x2 - x1)

# Calcula la pendiente de la línea perpendicular
def perpendicular_slope(m):
    if m == float('inf'):
        return 0  # La línea perpendicular a una línea vertical es horizontal
    elif m == 0:
        return float('inf')  # La línea perpendicular a una línea horizontal es vertical
    else:
        return -1 / m

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
        if m1 * m2 == -1:
            return 90  # Líneas perpendiculares, ángulo es 90 grados
        tan_theta = abs((m2 - m1) / (1 + m1 * m2))
        angle = math.degrees(math.atan(tan_theta))  
        return angle


def are_lines_about_120_degrees(m1, m2, error_margin=15):
    # Calcula el ángulo entre las dos líneas
    angle = calculate_angle_between_lines(m1, m2)
    # print('angle', angle)
    
    # Verifica si el ángulo es aproximadamente 120 grados
    # Tambien se compara con 60 por si se esta tomando el menor angulo entre las rectas
    # 60 es el angulo suplementario de 120
    return abs(angle - 120) <= error_margin or abs(angle - 60) <= error_margin

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

def determine_direction(rotorY, vertical_lines, margin_error=5):
    upper_lines = []
    lower_lines = []
    for line in vertical_lines:
        x1, y1, x2, y2 = line[0]
        if (y1 < rotorY or y2 < rotorY):
            upper_lines.append(line)
        if (y1 > rotorY or y2 > rotorY):
            lower_lines.append(line)

    if (not upper_lines) or (not lower_lines):
        return  None, None
    
    upper_line = avg_line(upper_lines)
    ux1, uy1, ux2, uy2 = upper_line[0]
    lower_line = avg_line(lower_lines)
    lx1, ly1, lx2, ly2 = lower_line[0]

    max_x_l = max(lx1, lx2)
    min_x_l = min(lx1, lx2)

    min_x_u = min(ux1, ux2)
    max_x_u = max(ux1, ux2)

    avg_dev = (abs(max_x_l - min_x_u) + abs(min_x_l - max_x_u))/2

    orientation = 0
    if abs(max_x_l - min_x_u) > margin_error and abs(min_x_l - max_x_u) > margin_error:
        if max_x_l < min_x_u:
            orientation = 1 # Sentido antihorario: 1
        elif min_x_l > max_x_u:
            orientation = -1 # Sentido horario: -1

    return (avg_dev * orientation)

def avg_line(lines):
    sum_x1, sum_y1, sum_x2, sum_y2 = 0, 0, 0, 0
    n = len(lines)

    for line in lines:
        x1, y1, x2, y2 = line[0]
        sum_x1 += x1
        sum_y1 += y1
        sum_x2 += x2
        sum_y2 += y2

    avg_x1 = sum_x1 / n
    avg_y1 = sum_y1 / n
    avg_x2 = sum_x2 / n
    avg_y2 = sum_y2 / n

    return np.array([[avg_x1, avg_y1, avg_x2, avg_y2]])


def findYShape(img, lines, img_name):
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
            cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2) # LINEAS Y INVERTIDA VERDE

        intersectionsAverageX = 0
        intersectionsAverageY = 0
        for (x, y) in intersections:
            cv2.circle(img, (x, y), 5, (0, 0, 255), -1) # PUNTOS INTERSECCION ROJO
            intersectionsAverageX += x
            intersectionsAverageY += y

        if intersections:
            rotorY = intersectionsAverageY / len(intersections)
            rotorX = intersectionsAverageX / len(intersections)
            intersectionsAverageX = intersectionsAverageX / len(intersections) / img.shape[1]
            intersectionsAverageY = intersectionsAverageY / len(intersections) / img.shape[0]
            cv2.circle(img, (int(intersectionsAverageX), int(intersectionsAverageY)), 5, (255, 0, 0), -1)
            percentageInImage = (x1 + x2) / 2 / img.shape[1]
            fieldOfView = math.degrees(CAMERA_FOV)

            vertical_lines = []
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    if is_vertical(x1, y1, x2, y2):
                        vertical_lines.append(line)

            angle = percentageInImage * fieldOfView - fieldOfView / 2
        cv2.imshow(img_name, img)
        cv2.waitKey(1)
        return y_inverted_found, rotorX, rotorY, angle, intersectionsAverageY, vertical_lines
    return None, None, None, None, None, None