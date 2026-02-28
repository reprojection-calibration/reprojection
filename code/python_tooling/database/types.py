from enum import Enum


class SensorType(str, Enum):
    Camera = "camera"
    Imu = "imu"
