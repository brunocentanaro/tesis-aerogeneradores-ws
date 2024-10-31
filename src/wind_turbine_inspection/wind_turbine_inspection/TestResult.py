import time
import json
import os
from px4_msgs.msg import VehicleLocalPosition, TrajectorySetpoint
import numpy as np


class TestResult:
    def __init__(self):
        self.initTimestamp = time.time()
        self.batteryUsage = []
        self.setPoints = []
        self.localPositions = []
        self.corrections = []
        self.isCompleted = False
        self.positionErrors = []
        self.initSimTime = -1
        self.endSimTime = 0

    def addBatteryUsage(self, batteryUsage):
        self.batteryUsage.append(batteryUsage)

    def addSetPoint(self, setPoint):
        self.setPoints.append(setPoint)

    def addLocalPosition(self, localPosition):
        self.localPositions.append(localPosition)

    def addCorrection(self, correction):
        self.corrections.append(correction)

    def addPositionError(self, positionError):
        self.positionErrors.append(positionError)

    def close(self, log):
        self.endTimestamp = time.time()
        self.duration = self.endTimestamp - self.initTimestamp
        self.isCompleted = True

        log.info(f"Test duration: {self.duration:.2f} seconds")
        log.info(f"Number of corrections: {len(self.corrections)}")
        log.info(f"Number of set points: {len(self.setPoints)}")
        medianError = np.median(self.positionErrors)
        stdDevError = np.std(self.positionErrors)
        log.info(
            f"Median position error: {medianError:.2f} m (std dev: {stdDevError:.2f})")

        data = {
            'initTimestamp': self.initTimestamp,
            'endTimestamp': self.endTimestamp,
            'medianError': medianError,
            'stdDevError': stdDevError,
            'duration': self.duration,
            'batteryUsage': self.batteryUsage,
            'corrections': self.corrections,
            'positionErrors': self.positionErrors,
            'initSimTime': self.initSimTime,
            'endSimTime': self.endSimTime,
            'simDuration': self.endSimTime - self.initSimTime
        }

        timestamp_str = time.strftime('%Y-%m-%d_%H-%M-%S')
        filename = f"test_result_{timestamp_str}.json"

        with open(filename, 'w') as f:
            json.dump(data, f, indent=4, cls=CustomJSONEncoder)

        absolute_path = os.path.abspath(filename)
        log.info(f"Test data saved to {absolute_path}")


class CustomJSONEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, TrajectorySetpoint):
            return trajectorySetpointToJSON(obj)
        elif isinstance(obj, VehicleLocalPosition):
            return localPositionToJSON(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, (np.float32, np.float64)):
            return float(obj)
        elif isinstance(obj, (np.int32, np.int64)):
            return int(obj)
        else:
            return super().default(obj)


def localPositionToJSON(localPositionMsg):
    return {
        'n': float(localPositionMsg.x),
        'e': float(localPositionMsg.y),
        'd': float(localPositionMsg.z),
        'vn': float(localPositionMsg.vx),
        've': float(localPositionMsg.vy),
        'vd': float(localPositionMsg.vz)
    }


def trajectorySetpointToJSON(trajectorySetpointMsg):
    return {
        'position': {
            'n': float(trajectorySetpointMsg.position[0]),
            'e': float(trajectorySetpointMsg.position[1]),
            'd': float(trajectorySetpointMsg.position[2])
        },
    }
