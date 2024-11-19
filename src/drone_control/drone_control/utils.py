import math
from geopy.distance import geodesic
import numpy as np
from sympy import Point3D, Segment3D

# Calculate the bearing (angle) between two geographical points (lat, lon)
def get_bearing(lat1, lon1, lat2, lon2):
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    dlon = lon2_rad - lon1_rad
    x = math.sin(dlon) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - \
        math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
    bearing = math.atan2(x, y)
    return bearing

# Get a coordinate on a straight line from the current position towards the wind turbine
# at a specified distance before reaching the target
def getCoordinateInLineToWindTurbineXDistanceBefore(
        lat1, lon1, lat2, lon2, distance):
    total_distance = geodesic((lat1, lon1), (lat2, lon2)).meters
    distance_to_new_point = total_distance - distance

    bearing = get_bearing(lat1, lon1, lat2, lon2)

    return abs(distance_to_new_point), bearing

# Rotate NED (North, East, Down) coordinates by a specified angle
def rotate_ned(north, east, down, angle):
    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])

    ned_coords = np.array([north, east, down])
    rotated_coords = np.dot(rotation_matrix, ned_coords)

    return rotated_coords[0], rotated_coords[1], rotated_coords[2]

# Calculate the distance from a point (current position) to a line segment (previous and current setpoints)
def get_distance_to_segment(previousSetpoint, currentSetpoint, currentPos):
    p1 = Point3D(previousSetpoint[0], previousSetpoint[1], previousSetpoint[2])
    p2 = Point3D(currentSetpoint[0], currentSetpoint[1], currentSetpoint[2])
    p3 = Point3D(currentPos[0], currentPos[1], currentPos[2])
    segment = Segment3D(p1, p2)
    return segment.distance(p3)
