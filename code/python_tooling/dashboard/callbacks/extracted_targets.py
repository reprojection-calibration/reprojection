from dash import MATCH, Input, Output, State, no_update

from dashboard.server import app

@app.callback(
    Output({"type": "extracted_targets", "sensor_name": MATCH}, "figure"),
    Input("sensor-content-container", "children"),
    State("sensor-selection-dropdown", "value"),
    State("raw-data-store", "data"),
    State({"type": "extracted_targets", "sensor_name": MATCH}, "figure"),
)
def update_image_size(_, sensor_name, raw_data, fig):
    if sensor_name is None or raw_data is None or fig is None:
        return no_update

    camera_info = raw_data[sensor_name].get("camera_info")
    if not camera_info:
        return no_update

    # TODO(Jack): Load target config from database and set the target points graph dimensions too. If we do this than
    # update the name of the function to reflect really what we are doing.

    width = camera_info.get("width")
    height = camera_info.get("height")

    # NOTE(Jack): We have subplots in the figure but the process of json dict serialization that dash does prevent us
    # from reforming a proper subplot figure here. Therefore, we have to acces the values directly here which is
    # looks extremely brittle to me and I am quite sure will be annoying to maintain over time. But for now we have no
    # other choice!
    #
    # Here "axis2" should be the extracted image figure on the right side.
    fig["layout"]["xaxis2"]["range"] = [0, width]
    fig["layout"]["xaxis2"]["autorange"] = False

    fig["layout"]["yaxis2"]["range"] = [0, height]
    fig["layout"]["yaxis2"]["autorange"] = False

    return fig


# TODO(Jack): Do not hardcode counter ID!
app.clientside_callback(
    """
    function(composite_id, frame_idx, step_name, raw_data) {
        if (!composite_id || frame_idx == null || !raw_data) {
            return dash_clientside.no_update;
        }
        
        const sensor_name = composite_id["sensor_name"];
        const targets = raw_data?.[sensor_name]?.measurements?.targets;
        if(!targets){
            return dash_clientside.no_update;
        }

        const keys = Object.keys(targets ?? {});
        const key = keys[frame_idx];
        if (!key) {
            throw new Error(`Invalid frame_idx: ${frame_idx}`);
        }
        
        const target = targets[key];
        
        const points = target.points.map(row => row.slice(0, 2));
        const pixels = target.pixels.map(row => row.slice(0, 2));
       
        const patch = new dash_clientside.Patch();
        patch.assign(['data', 0, 'x'], points.map(p => p[0]));
        patch.assign(['data', 0, 'y'], points.map(p => p[1]));
        patch.assign(['data', 1, 'x'], pixels.map(p => p[0]));
        patch.assign(['data', 1, 'y'], pixels.map(p => p[1]));
        patch.assign(['data', 0, 'marker'], {size: 12, color: "darkgray"});
        patch.assign(['data', 1, 'marker'], {size: 12, color: "darkgray"});
        
        const reprojection_errors = raw_data?.[sensor_name]?.reprojection_error?.[step_name];
        if(reprojection_errors && (key in reprojection_errors)){
            const reprojection_error = reprojection_errors[key]

            patch.assign(['data', 0, 'marker'], {
                size: 12,
                color: reprojection_error.map(p => Math.sqrt(p[0] * p[0] + p[1] * p[1])),
                colorscale: "Bluered",
                cmin: 0,
                cmax: 1,
            });
            patch.assign(['data', 1, 'marker'], {
                size: 12,
                color: reprojection_error.map(p => Math.sqrt(p[0] * p[0] + p[1] * p[1])),
                colorscale: "Bluered",
                cmin: 0,
                cmax: 1,
            });
            
            patch.assign(['data', 0, 'hovertemplate'], "x: %{x}<br>y: %{y}<br>error: %{marker.color:.3f}<extra></extra>");
            patch.assign(['data', 1, 'hovertemplate'], "x: %{x}<br>y: %{y}<br>error: %{marker.color:.3f}<extra></extra>");
        }
    
        return patch.build();
    }
    """,
    Output(
        {"type": "extracted_targets", "sensor_name": MATCH},
        "figure",
        allow_duplicate=True,
    ),
    Input({"type": "extracted_targets", "sensor_name": MATCH}, "id"),
    Input({"type": "slider", "sensor_name": MATCH}, "value"),
    Input("step-selector", "value"),
    State("raw-data-store", "data"),
    prevent_initial_call=True,
)
