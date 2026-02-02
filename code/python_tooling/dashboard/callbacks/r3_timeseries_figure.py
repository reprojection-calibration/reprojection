from dash import Input, Output, State

from dashboard.server import app
from dashboard.tools.r3_timeseries_figure import (
    R3TimeseriesFigureConfig,
    build_r3_timeseries_figure,
    build_r6_timeseries_figures,
    make_timeseries_annotation_clientside_callback,
    timeseries_plot,
)
from database.types import SensorType


# TODO IMU DATA DOES NOT HAVE POSE SELECTOR! Yet....
def register_build_r6_timeseries_figures_callback(
    fig1_id,
    fig2_id,
    sensor_dropdown_id,
    raw_data_store_id,
    sensor_type,
    fig1_config,
    fig2_config,
):
    # TODO(Jack): The pose type selector is currently only used for the camera data case - maybe in future we repurpose
    #  it to explicitly represent a state (i.e. optimized error or not) and then can use this for the imu data also.
    @app.callback(
        Output(fig1_id, "figure", allow_duplicate=True),
        Output(fig2_id, "figure", allow_duplicate=True),
        Input(sensor_dropdown_id, "value"),
        Input("pose-type-selector", "value"),
        State(raw_data_store_id, "data"),
        State("metadata-store", "data"),
        prevent_initial_call=True,
    )
    def build_r6_timeseries_figures_callback(sensor, pose_type, raw_data, metadata):
        # TODO(Jack): We should move these data quality checks into a testable method. These are nowhere tested.
        if (
            sensor is None
            or (pose_type is None and sensor_type == SensorType.Camera)
            or raw_data is None
            or metadata is None
        ):
            return {}, {}

        _, timestamps = metadata
        timestamps = timestamps[sensor_type]
        if sensor not in timestamps:
            return {}, {}

        # NOTE(Jack): Because sensor was in timestamps, sensor MUST also be in raw_data.
        # NOTE(Jack): # The initial condition assertion means that if sensor_type == Camera that pose_type MUST be set.
        return build_r6_timeseries_figures(
            timestamps[sensor],
            raw_data[sensor]["frames"],
            sensor_type,
            fig1_config,
            fig2_config,
            pose_type,
        )


# Camera calibration pose figures
camera_orientation_config = R3TimeseriesFigureConfig(
    "Orientation",
    "Axis Angle (rad)",
    "rx",
    "ry",
    "rz",
    -3.14,
    3.14,
)
camera_translation_config = R3TimeseriesFigureConfig(
    "Translation",
    "Meter (m)",
    "x",
    "y",
    "z",
    -2,
    2,
)
register_build_r6_timeseries_figures_callback(
    "camera-orientation-graph",
    "camera-translation-graph",
    "camera-sensor-dropdown",
    "raw-camera-data-store",
    SensorType.Camera,
    camera_orientation_config,
    camera_translation_config,
)

# Imu calibration measurement figures
imu_angular_velocity_config = R3TimeseriesFigureConfig(
    "Angular Velocty",
    "(rad/s)",
    "omega_x",
    "omega_y",
    "omega_z",
    -3.14,
    3.14,
)
imu_linear_acceleration_config = R3TimeseriesFigureConfig(
    "Linear Acceleration",
    "(m/s2)",
    "ax",
    "ay",
    "az",
    -10,
    10,
)
register_build_r6_timeseries_figures_callback(
    "imu-angular-velocity-graph",
    "imu-linear-acceleration-graph",
    "imu-sensor-dropdown",
    "raw-imu-data-store",
    SensorType.Imu,
    imu_angular_velocity_config,
    imu_linear_acceleration_config,
)


def register_timeseries_annotation_clientside_callback(
    fig1_id,
    fig2_id,
    slider_id,
    sensor_dropdown_id,
    sensor_type,
):
    app.clientside_callback(
        make_timeseries_annotation_clientside_callback(sensor_type),
        Output(fig1_id, "figure"),
        Output(fig2_id, "figure"),
        Input(slider_id, "value"),
        Input(sensor_dropdown_id, "value"),
        State("metadata-store", "data"),
        State(fig1_id, "figure"),
        State(fig2_id, "figure"),
    )


register_timeseries_annotation_clientside_callback(
    "camera-orientation-graph",
    "camera-translation-graph",
    "camera-frame-id-slider",
    "camera-sensor-dropdown",
    SensorType.Camera,
)

register_timeseries_annotation_clientside_callback(
    "imu-angular-velocity-graph",
    "imu-linear-acceleration-graph",
    "imu-frame-id-slider",
    "imu-sensor-dropdown",
    SensorType.Imu,
)
