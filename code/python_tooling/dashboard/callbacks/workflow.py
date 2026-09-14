from dash import Input, Output

from dashboard.server import app
from dashboard.tools.workflow import build_stage_overview, stage_navigation


@app.callback(
    Output("stage-selector", "options"),
    Output("stage-selector", "value"),
    Input("metadata-store", "data"),
)
def update_stages(metadata):
    return stage_navigation(metadata)


@app.callback(
    Output("stage-overview", "children"),
    Input("stage-selector", "value"),
    Input("metadata-store", "data"),
)
def update_stage_overview(stage_id, metadata):
    return build_stage_overview(stage_id, metadata)
