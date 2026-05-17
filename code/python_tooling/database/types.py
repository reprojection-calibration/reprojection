from enum import Enum


class SensorType(str, Enum):
    Camera = "camera"
    Imu = "imu"


class TargetType(str, Enum):
    Aprilgrid3 = "aprilgrid3"
    Checkerboard = "checkerboard"
    CircleGrid = "circle_grid"
