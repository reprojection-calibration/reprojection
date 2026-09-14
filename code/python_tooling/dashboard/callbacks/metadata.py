from dash import Input, Output

from dashboard.server import app
from dashboard.tools.metadata import build_sensor_metadata_layout, step_selector_options


@app.callback(
    Output("sensor-statistics-container", "children"),
    Output("step-selector", "options"),
    Output("step-selector", "value"),
    Input("sensor-selection-dropdown", "value"),
    Input("metadata-store", "data"),
    Input("workflow-data-store", "data"),
)
def update_sensor_metadata(asset_id, metadata, workflow_data):
    options, value = step_selector_options(asset_id, metadata)
    return (
        build_sensor_metadata_layout(asset_id, metadata, workflow_data),
        options,
        value,
    )
