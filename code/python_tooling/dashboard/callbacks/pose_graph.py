from dash import Input, Output, State
from dash.exceptions import PreventUpdate
from server import app
from tools.plot_pose_figure import plot_pose_figure
from tools.time_handling import extract_timestamps_and_poses_sorted


@app.callback(
    Output("pose-figure-store", "data"),
    Input("sensor-dropdown", "value"),
    Input("pose-type-selector", "value"),
    State("raw-data-store", "data"),
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
        return {
            "rotation": {},
            "translation": {},
        }

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

    return {
        "rotation": rot_fig,
        "translation": trans_fig,
    }


# ERROR IT WILL NOT UPDATE WHEN WE SELECT FROM OPTIMIZED/INITIAL
app.clientside_callback(
    """
    function(frame_idx, sensor, fig_store, processed_data, rot_fig, trans_fig) {
        // DOCUMENT
        // DOCUMENT
        // DOCUMENT
        // DOCUMENT
        // DOCUMENT
        const ctx = dash_clientside.callback_context;
        const triggered = dash_clientside.callback_context.triggered.map(t => t.prop_id);
        if (triggered.includes("pose-figure-store.data")) {
            if (fig_store && fig_store.rotation && fig_store.translation) {
                console.log("Store updated → refreshing figures");
                return [fig_store.rotation, fig_store.translation];
            }
        }
    
        // DOCUMENT
        // DOCUMENT
        // DOCUMENT
        // DOCUMENT
        // DOCUMENT
        if (!rot_fig || !trans_fig) {
            if (fig_store && fig_store.rotation && fig_store.translation) {
                return [fig_store.rotation, fig_store.translation];
            }
    
            const EMPTY_FIG = {
                data: [],
                layout: {
                    xaxis: { visible: true },
                    yaxis: { visible: true },
                    shapes: []
                }
            };
            return [EMPTY_FIG, EMPTY_FIG];
        }
    
        // ---- STEADY STATE FROM HERE ON ----
        if (!sensor || !processed_data) {
            return [rot_fig, trans_fig];
        }
    
        if (!processed_data[1] || !processed_data[1][sensor]) {
            return [rot_fig, trans_fig];
        }
    
        const timestamps = processed_data[1][sensor];
        if (!timestamps || timestamps.length <= frame_idx) {
            return [rot_fig, trans_fig];
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
    Input("pose-figure-store", "data"),
    State("processed-data-store", "data"),
    State("rotation-graph", "figure"),
    State("translation-graph", "figure"),
)
