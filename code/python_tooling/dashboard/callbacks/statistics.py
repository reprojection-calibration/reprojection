from dash import Input, Output, State

from dashboard.server import app
from dashboard.tools.statistics import build_sensor_statistics_div
from database.types import PoseType, SensorType


def register_statistics_display_update_callback(
    display_id, sensor_dropdown_id, sensor_type
):
    @app.callback(
        Output(display_id, "children"),
        Input(sensor_dropdown_id, "value"),
        State("metadata-store", "data"),
    )
    def update_sensor_statistics_display(sensor, metadata):
        return build_sensor_statistics_div(sensor, metadata, sensor_type)


register_statistics_display_update_callback(
    "camera-statistics-display", "camera-sensor-dropdown", SensorType.Camera
)
register_statistics_display_update_callback(
    "imu-statistics-display", "imu-sensor-dropdown", SensorType.Imu
)
