from dash import Input, Output

from dashboard.server import app
from dashboard.tools.metadata import build_sensor_metadata_layout, step_selector_options
from dashboard.tools.results import build_result_summary


@app.callback(
    Output("sensor-statistics-container", "children"),
    Output("step-selector", "options"),
    Output("step-selector", "value"),
    Input("sensor-selection-dropdown", "value"),
    Input("metadata-store", "data"),
    Input("workflow-data-store", "data"),
    Input("stage-selector", "value"),
)
def update_sensor_metadata(asset_id, metadata, workflow_data, stage_id):
    options, value = step_selector_options(asset_id, metadata, stage_id)
    return (
        build_sensor_metadata_layout(asset_id, metadata, workflow_data),
        options,
        value,
    )


@app.callback(
    Output("result-summary", "children"),
    Input("sensor-selection-dropdown", "value"),
    Input("step-selector", "value"),
    Input("metadata-store", "data"),
    Input("workflow-data-store", "data"),
    Input("stage-selector", "value"),
)
def update_result_summary(asset_id, step_id, metadata, workflow_data, stage_id):
    return build_result_summary(asset_id, step_id, metadata, workflow_data, stage_id)
