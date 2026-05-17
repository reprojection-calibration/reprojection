from enum import Enum


class SensorType(str, Enum):
    Camera = "camera"
    Imu = "imu"


class TargetType(str, Enum):
    Aprilgrid3 = "aprilgrid3"
    Checkerboard = "checkerboard"
    CircleGrid = "circle_grid"

class CameraModel(str, Enum):
    DoubleSphere = "double_sphere"
    Pinhole = "pinhole"
    PinholeRadtan4 = "pinhole_radtan4"
    UnifiedCameraModel = "unified_camera_model"