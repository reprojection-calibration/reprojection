from dash import Input, Output, State

from dashboard.server import app
from dashboard.tools.data_loading import (
    build_sensor_statistics_html,
    load_database,
    refresh_database_list,
    refresh_sensor_list,
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
    Output("raw-data-store", "data"),
    Output("metadata-store", "data"),
    Output("timestamp-store", "data"),
    Input("database-selection-dropdown", "value"),
)
def load_database_callback(db_file):
    return load_database(db_file)


@app.callback(
    Output("sensor-selection-dropdown", "options"),
    Output("sensor-selection-dropdown", "value"),
    Input("metadata-store", "data"),
)
def refresh_sensor_list_callback(metadata):
    return refresh_sensor_list(metadata)


@app.callback(
    Output("sensor-statistics-container", "children"),
    Input("sensor-selection-dropdown", "value"),
    State("metadata-store", "data"),
)
def update_sensor_statistics(sensor_name, metadata):
    return build_sensor_statistics_html(sensor_name, metadata)
