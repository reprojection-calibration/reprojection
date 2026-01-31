import os

from dash import Input, Output
from database.types import SensorType

from dashboard.server import app
from database.load_camera_calibration_data import (
    get_camera_calibration_data_statistics,
    get_indexable_timestamp_record,
    load_camera_calibration_data,
)

from database.load_imu_calibration_data import load_imu_calibration_data, get_imu_calibration_data_statistics

from database.types import PoseType


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
    Output("raw-camera-data-store", "data"),
    Output("raw-imu-data-store", "data"),
    Output("processed-data-store", "data"),
    Input("database-dropdown", "value"),
)
def load_database_to_store(db_file):
    if not db_file or not os.path.isfile(db_file):
        return None, None

    # We already check this when we load the db list, but it doesn't hurt to double-check :)
    if not db_file.endswith(".db3"):
        return None, None

    raw_camera_data = load_camera_calibration_data(db_file)
    raw_imu_data = load_imu_calibration_data(db_file)

    statistics = {
        SensorType.Camera: get_camera_calibration_data_statistics(raw_camera_data),
        SensorType.Imu: get_imu_calibration_data_statistics(raw_imu_data)
    }

    indexable_timestamps = {
        SensorType.Camera: get_indexable_timestamp_record(raw_camera_data),
        SensorType.Imu: get_indexable_timestamp_record(raw_imu_data)
    }

    return raw_camera_data, raw_imu_data, [statistics, indexable_timestamps],


def register_sensor_dropdown_callback(dropdown_id, sensor_type):
    @app.callback(
        Output(dropdown_id, "options"),
        Output(dropdown_id, "value"),
        Input("processed-data-store", "data"),
    )
    def refresh_sensor_dropdown_list(processed_data):
        if processed_data is None:
            return [], ""

        statistics, _ = processed_data
        sensor_statistics = statistics[sensor_type]
        sensor_names = sorted(sensor_statistics.keys())

        if len(sensor_names) == 0:
            return [], ""

        return sensor_names, sensor_names[0]


register_sensor_dropdown_callback("camera-sensor-dropdown", SensorType.Camera)
register_sensor_dropdown_callback("imu-sensor-dropdown", SensorType.Imu)
