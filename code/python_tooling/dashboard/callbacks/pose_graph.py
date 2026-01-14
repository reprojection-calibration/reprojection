from dash import Input, Output, State
from dash.exceptions import PreventUpdate

from dashboard.server import app
from dashboard.tools.plot_pose_figure import plot_pose_figure
from dashboard.tools.time_handling import extract_timestamps_and_poses_sorted


@app.callback(
    Output("rotation-graph", "figure", allow_duplicate=True),
    Output("translation-graph", "figure", allow_duplicate=True),
    Input("sensor-dropdown", "value"),
    Input("pose-type-selector", "value"),
    State("raw-data-store", "data"),
    prevent_initial_call=True,
)
def update_pose_graph(selected_sensor, pose_type, raw_data):
    # TODO(Jack): What is an effective way to actually use this mechanism? Having it at the start of every single
    #  callback does not feel right somehow. But managing when each input or state is available is not trivial! For
    #  example here the pose type always should have a default value, and technically the raw-data store should always
    #  be populated before the sensor dropdown triggers (because the sensor dropdown depends on the raw data being
    #  loaded). But what if any of those things change? And as this program scales these relationships will likely no
    #  longer be trackable. Does that mean the best option is to actually raise PreventUpdate at every callback?
    if selected_sensor is None or pose_type is None or raw_data is None:
        raise PreventUpdate

    # TODO(Jack): Technically this is not nice. To access a key (frames) without checking that it exists. However this
    # is so fundamental to the data structure I think I can be forgiven for this. Remove this todo later if it turns out
    # to be a nothing burger.
    frames = raw_data[selected_sensor]["frames"]
    if frames is None:
        raise PreventUpdate

    timestamps_ns, poses = extract_timestamps_and_poses_sorted(frames, pose_type)

    rotations = [d[:3] for d in poses]
    rot_fig = plot_pose_figure(
        timestamps_ns,
        rotations,
        "Orientation",
        "Axis Angle (rad)",
        x_name="rx",
        y_name="ry",
        z_name="rz",
    )

    translations = [d[3:] for d in poses]
    trans_fig = plot_pose_figure(
        timestamps_ns, translations, "Translation", "Meter (m)"
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
            line: {color: 'black', width: 1},
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
