from dash import Input, Output, State, html

from dashboard.server import app
from dashboard.tools.construct_layouts import camera_layout, imu_layout
from database.types import SensorType


# TODO(Jack): At this time do we really need timestamps?
@app.callback(
    Output("sensor-content-container", "children"),
    Input("sensor-selection-dropdown", "value"),
    State("raw-data-store", "data"),
)
def render_sensor_panel(sensor_name, raw_data):
    if sensor_name is None or raw_data is None:
        return html.P("Calibration data not loaded.")

    # TODO(Jack): Should we check that this exists here or at a certain point can we take this for a given?
    sensor_type = raw_data[sensor_name]["type"]

    if sensor_type == SensorType.Camera:
        return camera_layout(sensor_name, raw_data)
    elif sensor_type == SensorType.Imu:
        return imu_layout()
    else:
        return html.P(f"Unknown sensor type selected: {sensor_type}.")
