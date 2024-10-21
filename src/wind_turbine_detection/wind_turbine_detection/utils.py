import math
import cv2
import numpy as np
from itertools import combinations
from wind_turbine_detection.constants import CAMERA_FOV


def preproces_and_hough(image):
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Improve contrast
    gray = cv2.equalizeHist(gray)

    # Apply smoothing
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply Canny edge detector with adjusted thresholds
    edges = cv2.Canny(blurred, threshold1=50, threshold2=100, apertureSize=3)

    # Apply the Probabilistic Hough Transform with adjusted parameters
    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180,
        threshold=30,
        minLineLength=15,
        maxLineGap=20
    )

    return lines


def determine_direction_with_depth(y_inverted_found, depth_image, angle_rotated=None, epsilon=0.1):
    vertical_edge, left_edge, right_edge = None, None, None

    if angle_rotated is not None:
        m_alpha = math.tan(math.radians(90 - angle_rotated))

    for line in y_inverted_found:
        m = slope(line)
        
        if angle_rotated:
            if abs(m - m_alpha) < epsilon:
                vertical_edge = line
            elif m < 0:
                left_edge = line
            else:
                right_edge = line
        else:
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
        # Calculate the slope (m) and intersection (b) of both lines
        m_left = slope(left_edge)
        b_left = intercept(left_edge, m_left)

        m_right = slope(right_edge)
        b_right = intercept(right_edge, m_right)

        # Get the value of x for the y of lower_coincidence on both lines
        x_left = x_at_y(m_left, b_left, lower_y_coincidence,
                        lx1 if m_left == float('inf') else None)
        x_right = x_at_y(m_right, b_right, lower_y_coincidence,
                         rx1 if m_right == float('inf') else None)

        distance_left = get_distance_at_approx_x_point(
            x_left, lower_y_coincidence, depth_image)
        distance_right = get_distance_at_approx_x_point(
            x_right, lower_y_coincidence, depth_image)

        if distance_left is None or distance_right is None:
            return None

        avg_dev = (abs(distance_left - distance_right))

        orientation = 0
        if distance_left < distance_right:
            orientation = 1  # Counterclockwise direction: 1
        elif distance_left > distance_right:
            orientation = -1  # Clockwise: -1

        return (avg_dev * orientation)
    return None

# Calculate the y-intercept (b) of the equation y = mx + b
def intercept(line, m):
    if m == float('inf'):
        return None  # Vertical line, has no intersection on the y axis
    x1, y1, _, _ = line[0]
    return y1 - m * x1

# Calculates the value of x given a value of y in the equation y = mx + b
# o returns the constant value of x if the line is vertical
def x_at_y(m, b, y, vertical_x=None):
    if m == float('inf'):
        return vertical_x  # Vertical line, always has a constant x
    return (y - b) / m


def get_distance_at_approx_x_point(x, y, depth_image, max_margin=5):
    distance = get_distance_at_point(x, y, depth_image)
    if (distance is None):
        return None

    if distance > 0:
        return distance

    for margin in range(1, max_margin + 1):
        # Check the positions to the left and right of the point (x, y)
        for dx in [margin, -margin]:
            distance = get_distance_at_point(x + dx, y, depth_image)
            if distance is not None and distance > 0:
                return distance

        # # Check at the additional positions around the point (x, y)
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
    # Check if the point is within the image boundaries
    if 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
        # Gets the distance value at the point (x, y)
        distance = depth_image[y, x]
        return distance
    return None

# Determines if a line is vertical within a margin of error
def is_vertical(x1, y1, x2, y2, error_margin=15):
    return abs(x1 - x2) <= error_margin

# Determines if a line is horizontal within a margin of error
def is_horizontal(x1, y1, x2, y2, error_margin=15):
    return abs(y1 - y2) <= error_margin

# Find the highest point in a list of lines
def highest_point(lines):
    highest = None
    for line in lines:
        x1, y1, x2, y2 = line[0]
        if highest is None or y1 < highest[1]:
            highest = (x1, y1)
        if y2 < highest[1]:
            highest = (x2, y2)
    return highest

# Calculate the Euclidean distance between two points p1 and p2
def distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

# Calculate the slope of the line
def slope(line):
    x1, y1, x2, y2 = line[0]
    if x2 == x1:
        return float('inf')
    else:
        return (y2 - y1) / (x2 - x1)

# Calculate the slope of the perpendicular line
def perpendicular_slope(m):
    if m == float('inf'):
        return 0  # The line perpendicular to a vertical line is horizontal
    elif m == 0:
        # The line perpendicular to a horizontal line is vertical
        return float('inf')
    else:
        return -1 / m


def calculate_angle_between_lines(m1, m2):
    if m1 == float('inf') or m2 == float('inf'):
        # Case where one of the lines is vertical
        if m1 == float('inf') and m2 == float('inf'):
            return 0  # The two lines are parallel and vertical, angle is 0
        elif m1 == float('inf'):
            # m1 is vertical, calculate angle with m2
            angle = math.degrees(math.atan(abs(m2)))
            return 90 - angle  # Angle with respect to a horizontal line
        else:
            # m2 is vertical, calculate angle with m1
            angle = math.degrees(math.atan(abs(m1)))
            return 90 - angle  # Angle with respect to a horizontal line
    else:
        # General case where no line is vertical
        if m1 * m2 == -1:
            return 90  # Perpendicular lines, angle is 90 degrees
        tan_theta = abs((m2 - m1) / (1 + m1 * m2))
        angle = math.degrees(math.atan(tan_theta))
        return angle


def are_lines_about_120_degrees(m1, m2, error_margin=15):
    # Calculate the angle between the two lines
    angle = calculate_angle_between_lines(m1, m2)
    # print('angle', angle)

    # Check if the angle is approximately 120 degrees
    # It is also compared with 60 in case the smallest angle between the lines is being taken
    # 60 is the supplementary angle of 120
    return abs(angle - 120) <= error_margin or abs(angle - 60) <= error_margin

# Returns 3 lines with approx 120 degrees between them
# Prefers the ones with the highest vertical line
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
            else:
                if highest is None:
                    highest = (line1, line2, line3)
    return highest

# Find the intersection between two lines represented by (x1, y1, x2, y2)
def find_line_intersection(line1, line2, tolerance=0.1):
    x1, y1, x2, y2 = line1[0]
    x_1, y_1, x_2, y_2 = line2[0]

    xdiff = (x1 - x2, x_1 - x_2)
    ydiff = (y1 - y2, y_1 - y_2)

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if abs(div) < tolerance:
        return None  # The lines are almost parallel or coincident

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
        return None, None

    upper_line = avg_line(upper_lines)
    ux1, uy1, ux2, uy2 = upper_line[0]
    lower_line = avg_line(lower_lines)
    lx1, ly1, lx2, ly2 = lower_line[0]

    max_x_l = max(lx1, lx2)
    min_x_l = min(lx1, lx2)

    min_x_u = min(ux1, ux2)
    max_x_u = max(ux1, ux2)

    avg_dev = (abs(max_x_l - min_x_u) + abs(min_x_l - max_x_u)) / 2

    orientation = 0
    if abs(max_x_l - min_x_u) > margin_error and abs(min_x_l -
                                                     max_x_u) > margin_error:
        if max_x_l < min_x_u:
            orientation = 1  # Counterclockwise direction: 1
        elif min_x_l > max_x_u:
            orientation = -1  # Clockwise: -1

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
    y_inverted_found = y_inverted(lines)

    # Draw the detected lines on the image
    if y_inverted_found:
        intersections = []

        # Find the intersections
        for i in range(len(y_inverted_found)):
            for j in range(i + 1, len(y_inverted_found)):
                intersection = find_line_intersection(
                    y_inverted_found[i], y_inverted_found[j])
                if intersection:
                    intersections.append(intersection)

        # Draw the lines
        for line in y_inverted_found:
            x1, y1, x2, y2 = line[0]
            # LINES Y INVERTED GREEN
            cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        maxX = 0
        maxY = 0
        minX = img.shape[1]
        minY = img.shape[0]
        intersectionsAverageY = 0
        for (x, y) in intersections:
            cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
            if x > maxX:
                maxX = x
            if x < minX:
                minX = x
            if y > maxY:
                maxY = y
            if y < minY:
                minY = y

        if intersections:
            rotorY = (maxY + minY) / 2
            rotorX = (maxX + minX) / 2
            percentageRotorY = rotorY / img.shape[0]
            cv2.circle(img, (int(rotorX), int(rotorY)), 5, (255, 0, 0), -1)
            percentageInImage = rotorX / img.shape[1]
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
        return y_inverted_found, rotorX, rotorY, angle, percentageRotorY, percentageRotorY
    return None, None, None, None, None, None
