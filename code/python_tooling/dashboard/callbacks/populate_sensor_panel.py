from dash import Input, Output, html

from dashboard.server import app
from dashboard.tools.construct_layouts import camera_layout, imu_layout
from database.types import SensorType


@app.callback(
    Output("sensor-content-container", "children"),
    Input("sensor-selection-dropdown", "value"),
    Input("metadata-store", "data"),
)
def render_sensor_panel(asset_id, metadata):
    if asset_id is None or metadata is None:
        return html.P("Calibration data not loaded.")

    asset = next(
        (asset for asset in metadata.get("assets", []) if asset["id"] == asset_id), None
    )
    if asset is None:
        return html.P("Calibration data not loaded.")
    sensor_type = asset["type"]

    if sensor_type == SensorType.Camera:
        return camera_layout(asset_id)
    elif sensor_type == SensorType.Imu:
        return imu_layout(asset_id)
    else:
        return html.P(f"Unknown sensor type selected: {sensor_type}.")
