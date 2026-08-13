from dash import Input, Output

from dashboard.server import app
from dashboard.tools.data_loading import (
    load_database,
    refresh_database_list,
    refresh_sensor_list,
    refresh_workflow_list,
)


@app.callback(
    Output("database-selection-dropdown", "options"),
    Output("database-selection-dropdown", "value"),
    Input("database-directory-input", "value"),
    Input("refresh-database-list-button", "n_clicks"),
)
def refresh_database_list_callback(db_dir, _):
    return refresh_database_list(db_dir)


@app.callback(
    Output("workflow-selection-dropdown", "options"),
    Output("workflow-selection-dropdown", "value"),
    Input("database-selection-dropdown", "value"),
)
def refresh_workflow_list_callback(db_file):
    return refresh_workflow_list(db_file)


@app.callback(
    Output("raw-data-store", "data"),
    Output("metadata-store", "data"),
    Input("database-selection-dropdown", "value"),
    Input("workflow-selection-dropdown", "value"),
)
def load_database_callback(db_file, workflow_id):
    return load_database(db_file, workflow_id)


@app.callback(
    Output("sensor-selection-dropdown", "options"),
    Output("sensor-selection-dropdown", "value"),
    Input("metadata-store", "data"),
)
def refresh_sensor_list_callback(metadata):
    return refresh_sensor_list(metadata)
