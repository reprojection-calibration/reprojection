import plotly.graph_objects as go
from dash import Input, Output, State

from dashboard.server import app
from dashboard.tools.plot_r3_timeseries import (
    R3TimeseriesFigureConfig,
    build_r3_timeseries_figure,
    timeseries_plot,
)
from dashboard.tools.time_handling import extract_timestamps_and_poses_sorted
from database.types import SensorType


# TODO IMU DATA DOES NOT HAVE POSE SELECTOR! Yet....
def register_r3_timeseries_plot_callback(
    fig1_id,
    fig2_id,
    sensor_dropdown_id,
    raw_data_store_id,
    sensor_type,
    fig1_config,
    fig2_config,
):
    # NOTE(Jack): This is a function of pure convenience. It just so happens that we need to plot two sets of three values,
    # both indexed by the same time. If this common coincidental requirement did not exist, then this function would not
    # exist.
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
    def init_pose_graph_figures(sensor, pose_type, raw_data, processed_data):
        if (
            sensor is None
            or (pose_type is None and sensor_type == SensorType.Camera)
            or raw_data is None
            or processed_data is None
        ):
            return {}, {}

        # NOTE(Jack): No matter what, we have the timestamps of all the possible frames because of the camera table foreign
        # key constraint. Even if we have no poses in the raw data we can still at least plot the properly sized and ranged
        # x-axis for that sensor. This looks good because the figure is configured even when no data is available, and
        # the x-axis range is fixed here, which means that if for example the optimized poses are only available for the
        # first half, it will be obvious to the user because the axis has not autofitted to the shorter timespan.
        _, indexable_timestamps = processed_data
        indexable_timestamps = indexable_timestamps[SensorType.Camera]
        if sensor not in indexable_timestamps:
            return {}, {}
        fig = timeseries_plot(indexable_timestamps[sensor])

        if sensor not in raw_data:
            raise RuntimeError(
                f"The sensor {sensor} was present in processed_data.indexable_timestamps but not in raw_data. That should never happen.",
            )

        if "frames" not in raw_data[sensor]:
            raise RuntimeError(
                f"The 'frames' key is not present in the raw data store for sensor {sensor}. That should never happen.",
            )

        # TODO(Jack): This is kind of a hack to handle extracting the data from the underlying raw data store. As long
        #  the raw data is rows of length 6, then we can probably keep this convention. If it gets more complicated than
        #  that (i.e. totally different storage ways), then we might need to think about more eloquent solutions.
        data_extractor = None
        if sensor_type == SensorType.Camera:
            data_extractor = lambda f: f["poses"][pose_type]
        elif sensor_type == SensorType.Imu:
            data_extractor = lambda f: f["imu_measurement"]
        else:
            raise RuntimeError(
                f"Invalid 'sensor_type' {sensor_type}. That should never happen.",
            )

        frames = raw_data[sensor]["frames"]
        timestamps_ns, data = extract_timestamps_and_poses_sorted(
            frames, data_extractor
        )

        if len(data) == 0:
            # Returns empty plots with properly labeled and ranged x-axis
            return fig, fig

        # TODO(Jack): We are hardcoding in the fact here that the underlying data is a nx6 list of lists
        fig1_data = [d[:3] for d in data]
        fig1 = build_r3_timeseries_figure(
            timestamps_ns,
            fig1_data,
            fig1_config,
            fig=go.Figure(fig),  # Deep copy to prevent edit in place
        )

        fig2_data = [d[3:] for d in data]
        fig2 = build_r3_timeseries_figure(
            timestamps_ns, fig2_data, fig2_config, fig=go.Figure(fig)
        )

        return fig1, fig2


camera_orientation_config = R3TimeseriesFigureConfig(
    "Orientation", "Axis Angle (rad)", "rx", "ry", "rz", -3.14, 3.14
)
camera_translation_config = R3TimeseriesFigureConfig(
    "Translation", "Meter (m)", "x", "y", "z", -2, 2
)

register_r3_timeseries_plot_callback(
    "camera-orientation-graph",
    "camera-translation-graph",
    "camera-sensor-dropdown",
    "raw-camera-data-store",
    SensorType.Camera,
    camera_orientation_config,
    camera_translation_config,
)

app.clientside_callback(
    """
    function(frame_idx, sensor, processed_data, rot_fig, trans_fig) {
        if (!sensor || !processed_data || !rot_fig || !trans_fig) {
            return [dash_clientside.no_update, dash_clientside.no_update];
        }
    
        // TODO(Jack): Do we need to protect against "camera" being available here, or can we take that for granted?
        const camera_processed_data = processed_data[1]["camera"]
        if (!camera_processed_data || !camera_processed_data[sensor]) {
            return [dash_clientside.no_update, dash_clientside.no_update];
        }
    
        const timestamps = camera_processed_data[sensor];
        if (!timestamps || timestamps.length <= frame_idx) {
            return [dash_clientside.no_update, dash_clientside.no_update];
        }
    
        const timestamp_0_ns = BigInt(timestamps[0]);
        const timestamp_i_ns = BigInt(timestamps[frame_idx]);
        const local_time_s = Number(timestamp_i_ns - timestamp_0_ns) / 1e9;
    
        // NOTE(Jack): The "paper" coordinate system goes from 0 to 1 to cover the entire figure, so we set yref to 
        // "paper" so that the y0=0 and y1=1 dimensions will draw a vertical line the entire figure height. 
        const new_shape = {
            type: 'rect',
            xref: 'x',
            yref: 'paper',
            x0: local_time_s,
            x1: local_time_s,
            y0: 0,
            y1: 1,
            line: {
                color: 'black',
                width: 1
            },
        };
    
        const new_annotation = {
            x: local_time_s,
            y: 1,
            xref: 'x',
            yref: 'paper',
            text: `${frame_idx}`,
            showarrow: false,
            yanchor: 'bottom',
            xanchor: 'center',
            font: {
                color: 'white',
                size: 12
            },
            bgcolor: 'rgba(10,10,10,0.7)',
        };
    
        // WARN(Jack): This might overwrite other pre-existing shapes that we add later!
        patch = new dash_clientside.Patch;
        patch.assign(['layout', 'shapes'], [new_shape]);
        patch.assign(['layout', 'annotations'], [new_annotation]);
    
        return [patch.build(), patch.build()];
    }
    """,
    Output("camera-orientation-graph", "figure"),
    Output("camera-translation-graph", "figure"),
    Input("camera-frame-id-slider", "value"),
    Input("camera-sensor-dropdown", "value"),
    State("processed-data-store", "data"),
    State("camera-orientation-graph", "figure"),
    State("camera-translation-graph", "figure"),
)
