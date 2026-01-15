import plotly.graph_objects as go
from dash import Input, Output, State

from dashboard.server import app
from dashboard.tools.plot_pose_figure import plot_pose_figure, timeseries_plot
from dashboard.tools.time_handling import extract_timestamps_and_poses_sorted


@app.callback(
    Output("rotation-graph", "figure", allow_duplicate=True),
    Output("translation-graph", "figure", allow_duplicate=True),
    Input("sensor-dropdown", "value"),
    Input("pose-type-selector", "value"),
    State("raw-data-store", "data"),
    State("processed-data-store", "data"),
    prevent_initial_call=True,
)
def init_pose_graph_figures(sensor, pose_type, raw_data, processed_data):
    if (
        sensor is None
        or pose_type is None
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

    # TODO(Jack): Is this error check here meaningful or valid at all? Or needed? What are we actually preventing here?
    frames = raw_data[sensor]["frames"]
    if frames is None:
        return fig, fig

    timestamps_ns, poses = extract_timestamps_and_poses_sorted(frames, pose_type)

    rotations = [d[:3] for d in poses]
    rot_fig = plot_pose_figure(
        timestamps_ns,
        rotations,
        "Orientation",
        "Axis Angle (rad)",
        fig=go.Figure(fig),  # Deep copy to prevent edit in place
        x_name="rx",
        y_name="ry",
        z_name="rz",
    )

    translations = [d[3:] for d in poses]
    trans_fig = plot_pose_figure(
        timestamps_ns, translations, "Translation", "Meter (m)", fig=go.Figure(fig)
    )

    return rot_fig, trans_fig


app.clientside_callback(
    """
    function(frame_idx, sensor, processed_data, rot_fig, trans_fig) {
        if (!sensor || !processed_data || !rot_fig || !trans_fig) {
            return [dash_clientside.no_update, dash_clientside.no_update];
        }
    
        if (!processed_data[1] || !processed_data[1][sensor]) {
            return [dash_clientside.no_update, dash_clientside.no_update];
        }
    
        const timestamps = processed_data[1][sensor];
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
    Output("rotation-graph", "figure"),
    Output("translation-graph", "figure"),
    Input("frame-id-slider", "value"),
    Input("sensor-dropdown", "value"),
    State("processed-data-store", "data"),
    State("rotation-graph", "figure"),
    State("translation-graph", "figure"),
)
