from dash import Input, Output, State

from dashboard.server import app
from dashboard.tools.metadata import build_sensor_metadata_layout


@app.callback(
    Output("sensor-statistics-container", "children"),
    Input("sensor-selection-dropdown", "value"),
    State("metadata-store", "data"),
)
def update_sensor_metadata(sensor_name, metadata):
    return build_sensor_metadata_layout(sensor_name, metadata)
