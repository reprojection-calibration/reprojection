import os

from dash import Input, Output

from dashboard.server import app
from dashboard.tools.data_loading import refresh_sensor_list
from database.load_camera_calibration_data import (
    get_camera_calibration_data_statistics,
    get_indexable_timestamp_record,
    load_camera_calibration_data,
)
from database.load_imu_calibration_data import (
    get_imu_calibration_data_statistics,
    load_imu_calibration_data,
)
from database.types import PoseType, SensorType


@app.callback(
    Output("database-dropdown", "options"),
    Output("database-dropdown", "value"),
    Input("database-directory-input", "value"),
    Input("refresh-database-list-button", "n_clicks"),
)
def refresh_database_list_callback(db_dir, _):
    if not os.path.exists(db_dir):
        return [], ""

    database_files = []
    for file_name in sorted(os.listdir(db_dir)):
        if not file_name.endswith(".db3"):
            continue

        full_path = os.path.join(db_dir, file_name)
        database_files.append(
            {
                "label": file_name,
                "value": full_path,
            }
        )

    if len(database_files) == 0:
        return [], ""

    return database_files, database_files[0]["value"]


# TODO(Jack): When we load a new database we should reset the slider to zero!
@app.callback(
    Output("raw-camera-data-store", "data"),
    Output("raw-imu-data-store", "data"),
    Output("metadata-store", "data"),
    Input("database-dropdown", "value"),
)
def load_database_to_stores_callback(db_file):
    if not db_file or not os.path.isfile(db_file):
        return None, None, None

    raw_camera_data = load_camera_calibration_data(db_file)
    raw_imu_data = load_imu_calibration_data(db_file)

    statistics = {
        SensorType.Camera: get_camera_calibration_data_statistics(raw_camera_data),
        SensorType.Imu: get_imu_calibration_data_statistics(raw_imu_data),
    }

    timestamps = {
        SensorType.Camera: get_indexable_timestamp_record(raw_camera_data),
        SensorType.Imu: get_indexable_timestamp_record(raw_imu_data),
    }

    return (
        raw_camera_data,
        raw_imu_data,
        [statistics, timestamps],
    )


def register_refresh_sensor_list_callback(dropdown_id, sensor_type):
    @app.callback(
        Output(dropdown_id, "options"),
        Output(dropdown_id, "value"),
        Input("metadata-store", "data"),
    )
    def refresh_sensor_list_callback(metadata):
        return refresh_sensor_list(metadata, sensor_type)


# NOTE(Jack): This comment belongs somewhere more central! But... The current design has some design points which are
# worth writing down. The most important points are:
#
#   (1) We the register_* pattern to prevent code duplication. When I added the IMU data visualization I found that I
#       wanted many of the same things that I had for the camera calibration visualization. Instead of copy and pasting
#       or hacking together similar code, I made this dependency explicit with the callback register_* pattern. This
#       means for completely common functionality like the sensor list dropdown, or slider index bar, we can initialize
#       them in the same exact manner by simply specifying the appropriate component IDs and SensorType.
register_refresh_sensor_list_callback("camera-sensor-dropdown", SensorType.Camera)
register_refresh_sensor_list_callback("imu-sensor-dropdown", SensorType.Imu)
