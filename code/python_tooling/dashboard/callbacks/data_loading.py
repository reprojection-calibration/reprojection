import os

from dash import Input, Output
from server import DB_DIR, app

from database.load_camera_calibration_data import (
    get_camera_calibration_data_statistics, get_indexable_timestamp_record,
    load_camera_calibration_data)


@app.callback(
    Output("database-dropdown", "options"),
    Output("database-dropdown", "value"),
    Input("refresh-database-list-button", "n_clicks"),
)
def refresh_database_list(_):
    if not os.path.exists(DB_DIR):
        return [], ""

    database_names = sorted([f for f in os.listdir(DB_DIR) if f.endswith(".db3")])
    if len(database_names) == 0:
        return [], ""

    return database_names, database_names[0]


# TODO(Jack): When we load a new database we should reset the slider to zero!
@app.callback(
    Output("raw-data-store", "data"),
    Output("processed-data-store", "data"),
    Input("database-dropdown", "value"),
)
def load_database_to_store(db_file):
    if not db_file:
        return None

    db_path = DB_DIR + db_file
    if not os.path.isfile(db_path):
        return None

    raw_data = load_camera_calibration_data(db_path)
    statistics = get_camera_calibration_data_statistics(raw_data)
    indexable_timestamps = get_indexable_timestamp_record(raw_data)

    # TODO(Jack): visualize the statistics in a data panel!
    return raw_data, [statistics, indexable_timestamps]


# TODO(Jack): is this really also a data loading callback? Seems well related but maybe not 100%
@app.callback(
    Output("sensor-dropdown", "options"),
    Output("sensor-dropdown", "value"),
    Input("processed-data-store", "data"),
)
def refresh_sensor_list(data):
    if not data:
        return [], None

    statistics, _ = data

    # We use a set here (e.g. the {} brackets) to enforce uniqueness
    sensor_names = sorted(list({sensor_name for sensor_name in statistics.keys()}))
    if len(sensor_names) == 0:
        return [], ""

    return sensor_names, sensor_names[0]
