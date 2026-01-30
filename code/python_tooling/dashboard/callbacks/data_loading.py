import os

from dash import Input, Output

from dashboard.server import app
from database.load_camera_calibration_data import (
    get_camera_calibration_data_statistics,
    get_indexable_timestamp_record,
    load_camera_calibration_data,
)

# TODO HACK AT INITIAL STAGE OF IMU DATA INTEGRATION!
from database.load_imu_data import imu_data_df_to_imu_calibration_data, load_imu_data_df


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
    Output("imu-data-store", "data"),
    Input("database-dropdown", "value"),
)
def load_database_to_store(db_file):
    if not db_file or not os.path.isfile(db_file):
        return None, None

    # We already check this when we load the db list, but it doesn't hurt to double-check :)
    if not db_file.endswith(".db3"):
        return None, None

    raw_data = load_camera_calibration_data(db_file)
    statistics = get_camera_calibration_data_statistics(raw_data)
    indexable_timestamps = get_indexable_timestamp_record(raw_data)

    # TODO HACK AT INITIAL STAGE OF IMU DATA INTEGRATION!
    imu_df = load_imu_data_df(db_file)
    imu_raw_data = imu_data_df_to_imu_calibration_data(imu_df)

    return raw_data, [statistics, indexable_timestamps], imu_raw_data


@app.callback(
    Output("sensor-dropdown", "options"),
    Output("sensor-dropdown", "value"),
    Input("processed-data-store", "data"),
)
def refresh_sensor_list(processed_data):
    if processed_data is None:
        return [], ""

    statistics, _ = processed_data
    sensor_names = sorted(statistics.keys())

    if len(sensor_names) == 0:
        return [], ""

    return sensor_names, sensor_names[0]
