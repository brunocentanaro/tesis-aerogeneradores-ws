import math
from geopy.distance import geodesic
import numpy as np

def get_bearing(lat1, lon1, lat2, lon2):
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    dlon = lon2_rad - lon1_rad
    x = math.sin(dlon) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
    bearing = math.atan2(x, y)
    return bearing

def getCoordinateInLineToWindTurbineXDistanceBefore(lat1, lon1, lat2, lon2, distance):
    drone_coord = (math.radians(lat1), math.radians(lon1))
    destination_coord = (math.radians(lat2), math.radians(lon2))
    total_distance = geodesic((lat1, lon1), (lat2, lon2)).meters
    distance_to_new_point = total_distance - distance

    bearing = get_bearing(lat1, lon1, lat2, lon2)

    return abs(distance_to_new_point), bearing


def rotate_ned(north, east, down, angle):
    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])
    
    ned_coords = np.array([north, east, down])
    rotated_coords = np.dot(rotation_matrix, ned_coords)
    
    return rotated_coords[0], rotated_coords[1], rotated_coords[2]