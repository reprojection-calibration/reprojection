import plotly.graph_objects as go
from dash import Input, Output, State

from dashboard.server import app
from dashboard.tools.plot_r3_timeseries import (
    R3TimeseriesFigureConfig,
    build_r3_timeseries_figure,
    timeseries_plot,
)
from dashboard.tools.time_handling import extract_timestamps_and_r6_data_sorted
from database.types import SensorType


# NOTE(Jack): This is a function of pure convenience. It just so happens that we need to plot two sets of three values,
# both indexed by the same time. If this common coincidental requirement did not exist, then this function would not
# exist.
def plot_two_common_r3_timeseries(
    all_timestamps_ns, frames, sensor_type, fig1_config, fig2_config, pose_type
):
    if sensor_type == SensorType.Camera:
        data_extractor = lambda f: f["poses"][pose_type]
    elif sensor_type == SensorType.Imu:
        data_extractor = lambda f: f["imu_measurement"]
    else:
        raise RuntimeError(
            f"Invalid 'sensor_type' {sensor_type}. That should never happen.",
        )

    # Build the plots using all timestamps so that even if there is no r3 data to plot below we can return figures with
    # properly sized x-axes
    fig = timeseries_plot(all_timestamps_ns)

    timestamps_ns, data = extract_timestamps_and_r6_data_sorted(frames, data_extractor)
    if len(data) == 0:
        return fig, fig

    # TODO(Jack): We are hardcoding in the fact here that the underlying data is a nx6 list of lists! Hacky.
    # TODO USE NUMPY!
    # NOTE(Jack): We deep copy like go.Figure(fig) to create to independent figures to prevent editing in place.
    fig1_data = [d[:3] for d in data]
    fig1 = build_r3_timeseries_figure(
        timestamps_ns,
        fig1_data,
        fig1_config,
        go.Figure(fig),
    )

    fig2_data = [d[3:] for d in data]
    fig2 = build_r3_timeseries_figure(
        timestamps_ns,
        fig2_data,
        fig2_config,
        go.Figure(fig),
    )

    return fig1, fig2


# TODO IMU DATA DOES NOT HAVE POSE SELECTOR! Yet....
def register_timeseries_figure_builder_callback(
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
        State("processed-data-store", "data"),
        prevent_initial_call=True,
    )
    def build_timeseries_figures(sensor, pose_type, raw_data, processed_data):
        if (
            sensor is None
            or (pose_type is None and sensor_type == SensorType.Camera)
            or raw_data is None
            or processed_data is None
        ):
            return {}, {}

        _, timestamps = processed_data
        timestamps = timestamps[sensor_type]
        if sensor not in timestamps:
            return {}, {}

        # NOTE(Jack): Because sensor was in timestamps, sensor MUST also be in raw_data.
        # NOTE(Jack): # The initial condition assertion means that if sensor_type == Camera that pose_type MUST be set.
        return plot_two_common_r3_timeseries(
            timestamps[sensor],
            raw_data[sensor]["frames"],
            sensor_type,
            fig1_config,
            fig2_config,
            pose_type,
        )


camera_orientation_config = R3TimeseriesFigureConfig(
    "Orientation", "Axis Angle (rad)", "rx", "ry", "rz", -3.14, 3.14
)
camera_translation_config = R3TimeseriesFigureConfig(
    "Translation", "Meter (m)", "x", "y", "z", -2, 2
)
register_timeseries_figure_builder_callback(
    "camera-orientation-graph",
    "camera-translation-graph",
    "camera-sensor-dropdown",
    "raw-camera-data-store",
    SensorType.Camera,
    camera_orientation_config,
    camera_translation_config,
)

imu_angular_velocity_config = R3TimeseriesFigureConfig(
    "Angular Velocty", "(rad/s)", "omega_x", "omega_y", "omega_z", -3.14, 3.14
)
imu_linear_acceleration_config = R3TimeseriesFigureConfig(
    "Linear Acceleration", "(m/s2)", "ax", "ay", "az", -10, 10
)
register_timeseries_figure_builder_callback(
    "imu-angular-velocity-graph",
    "imu-linear-acceleration-graph",
    "imu-sensor-dropdown",
    "raw-imu-data-store",
    SensorType.Imu,
    imu_angular_velocity_config,
    imu_linear_acceleration_config,
)


# NOTE(Jack): Unfortunately the only way to achieve the parameterization of the clientside callbacks is to generate the
# code. All we want to do is specify the sensor type!
def make_r3_timeseries_annotation_clientside_callback(sensor_type):
    return f"""
    function(frame_idx, sensor, processed_data, rot_fig, trans_fig) {{
        if (!sensor || !processed_data || !rot_fig || !trans_fig) {{
            return [dash_clientside.no_update, dash_clientside.no_update];
        }}
    
        const sensor_processed_data = processed_data[1]["{sensor_type.value}"]
        if (!sensor_processed_data || !sensor_processed_data[sensor]) {{
            return [dash_clientside.no_update, dash_clientside.no_update];
        }}
    
        const timestamps = sensor_processed_data[sensor];
        if (!timestamps || timestamps.length <= frame_idx) {{
            return [dash_clientside.no_update, dash_clientside.no_update];
        }}
    
        const timestamp_0_ns = BigInt(timestamps[0]);
        const timestamp_i_ns = BigInt(timestamps[frame_idx]);
        const local_time_s = Number(timestamp_i_ns - timestamp_0_ns) / 1e9;
    
        // NOTE(Jack): The "paper" coordinate system goes from 0 to 1 to cover the entire figure, so we set yref to 
        // "paper" so that the y0=0 and y1=1 dimensions will draw a vertical line the entire figure height. 
        const new_shape = {{
            type: 'rect',
            xref: 'x',
            yref: 'paper',
            x0: local_time_s,
            x1: local_time_s,
            y0: 0,
            y1: 1,
            line: {{
                color: 'black',
                width: 1
            }},
        }};
    
        const new_annotation = {{
            x: local_time_s,
            y: 1,
            xref: 'x',
            yref: 'paper',
            text: `${{frame_idx}}`,
            showarrow: false,
            yanchor: 'bottom',
            xanchor: 'center',
            font: {{
                color: 'white',
                size: 12
            }},
            bgcolor: 'rgba(10,10,10,0.7)',
        }};
    
        // WARN(Jack): This might overwrite other pre-existing shapes that we add later!
        patch = new dash_clientside.Patch;
        patch.assign(['layout', 'shapes'], [new_shape]);
        patch.assign(['layout', 'annotations'], [new_annotation]);
    
        return [patch.build(), patch.build()];
    }}
    """


def register_r3_timeseries_annotation_clientside_callback(
    fig1_id,
    fig2_id,
    slider_id,
    sensor_dropdown_id,
    sensor_type,
):
    app.clientside_callback(
        make_r3_timeseries_annotation_clientside_callback(sensor_type),
        Output(fig1_id, "figure"),
        Output(fig2_id, "figure"),
        Input(slider_id, "value"),
        Input(sensor_dropdown_id, "value"),
        State("processed-data-store", "data"),
        State(fig1_id, "figure"),
        State(fig2_id, "figure"),
    )


register_r3_timeseries_annotation_clientside_callback(
    "camera-orientation-graph",
    "camera-translation-graph",
    "camera-frame-id-slider",
    "camera-sensor-dropdown",
    SensorType.Camera,
)

register_r3_timeseries_annotation_clientside_callback(
    "imu-angular-velocity-graph",
    "imu-linear-acceleration-graph",
    "imu-frame-id-slider",
    "imu-sensor-dropdown",
    SensorType.Imu,
)
