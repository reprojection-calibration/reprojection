from dash import Input, Output, State

from dashboard.server import app
from dashboard.tools.statistics import build_sensor_statistics_html


@app.callback(
    Output("sensor-statistics-container", "children"),
    Input("sensor-selection-dropdown", "value"),
    State("metadata-store", "data"),
)
def update_sensor_statistics(sensor_name, metadata):
    return build_sensor_statistics_html(sensor_name, metadata)
