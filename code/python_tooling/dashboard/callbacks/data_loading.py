import os

from dash import Input, Output

from dashboard.server import app
from database.load_camera_calibration_data import (
    get_camera_calibration_data_statistics,
    get_indexable_timestamp_record,
    load_camera_calibration_data,
)


# TODO(Jack): Add selected dir input!
@app.callback(
    Output("database-dropdown", "options"),
    Output("database-dropdown", "value"),
    Input("database-directory-input", "value"),
    Input("refresh-database-list-button", "n_clicks"),
)
def refresh_database_list(db_dir, _):
    if not os.path.exists(db_dir):
        return [], ""

    options = []
    for file_name in sorted(os.listdir(db_dir)):
        if not file_name.endswith(".db3"):
            continue

        full_path = os.path.join(db_dir, file_name)
        options.append(
            {
                "label": file_name,
                "value": full_path,
            }
        )

    if len(options) == 0:
        return [], ""

    return options, options[0]["value"]


# TODO(Jack): When we load a new database we should reset the slider to zero!
@app.callback(
    Output("raw-data-store", "data"),
    Output("processed-data-store", "data"),
    Input("database-dropdown", "value"),
)
def load_database_to_store(db_file):
    if not db_file:
        return None, None

    if not os.path.isfile(db_file):
        return None, None

    raw_data = load_camera_calibration_data(db_file)
    statistics = get_camera_calibration_data_statistics(raw_data)
    indexable_timestamps = get_indexable_timestamp_record(raw_data)

    return raw_data, [statistics, indexable_timestamps]


@app.callback(
    Output("sensor-dropdown", "options"),
    Output("sensor-dropdown", "value"),
    Input("processed-data-store", "data"),
)
def refresh_sensor_list(processed_data):
    if not processed_data:
        return [], ""

    statistics, _ = processed_data

    # We use a set here (e.g. the {} brackets) to enforce uniqueness
    sensor_names = sorted(list({sensor_name for sensor_name in statistics.keys()}))
    if len(sensor_names) == 0:
        return [], ""

    return sensor_names, sensor_names[0]
