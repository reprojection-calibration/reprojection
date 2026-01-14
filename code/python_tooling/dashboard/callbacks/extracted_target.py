import plotly.graph_objects as go
from dash import Input, Output, State

from dashboard.server import IMAGE_DIMENSIONS, app


# TODO(Jack): Technically we only need the sensor name to get the frame-id-slider output, but this does not necessarily
#  have anything to do with configuring the figures initially. It might make sense to move the frame id slider
#  dependency to another place that is more related or independent than here.
@app.callback(
    Output("targets-xy-graph", "figure", allow_duplicate=True),
    Output("targets-pixels-graph", "figure", allow_duplicate=True),
    Output("frame-id-slider", "max"),
    Input("sensor-dropdown", "value"),
    State("processed-data-store", "data"),
    prevent_initial_call=True,
)
def init_extracted_target_figures(sensor, data):
    if not sensor or not data:
        return {}, {}, 0

    # TODO(Jack): Confirm/test ALL axes properties (ranges, names, orders etc.) None of this has been checked! Even the
    #  coordinate conventions of the pixels and points needs to be checked!
    # TODO(Jack): Eliminate copy and paste here in this method! We basically do the same thing twice.
    # TODO(Jack): We have now copy and pasted in several places that the marker size for xy_fig is 12 and for pixel_fig
    #  is 6. This is a hack! We need to auto scale all dimensions and all marker sizes! Or at least make them more
    #  generic.
    xy_fig = go.Figure()
    xy_fig.add_trace(
        go.Scatter(
            x=[],
            y=[],
            mode="markers",
            marker=dict(size=12),
            hovertemplate="x: %{x}<br>"
            + "y: %{y}<br>"
            + "error: %{marker.color:.3f}<extra></extra>",
        )
    )
    xy_fig.update_layout(
        title="Target Points (XY)",
        xaxis=dict(
            range=[-0.1, 0.76],  # ERROR(Jack): Do not hardcode or use global
            title=dict(text="x"),
            constrain="domain",
        ),
        yaxis=dict(
            range=[-0.1, 0.76],  # ERROR(Jack): Do not hardcode or use global
            title=dict(text="y"),
            scaleanchor="x",
        ),
    )

    pixel_fig = go.Figure()
    pixel_fig.add_trace(
        go.Scatter(
            x=[],
            y=[],
            mode="markers",
            marker=dict(size=6),
            hovertemplate="x: %{x}<br>"
            + "y: %{y}<br>"
            + "error: %{marker.color:.3f}<extra></extra>",
        )
    )
    pixel_fig.update_layout(
        title="Extracted Pixel Features",
        xaxis=dict(
            range=[
                0,
                IMAGE_DIMENSIONS[0],
            ],  # ERROR(Jack): Do not hardcode or use global
            title=dict(text="u"),
            constrain="domain",
        ),
        yaxis=dict(
            range=[IMAGE_DIMENSIONS[1], 0],  # invert Y for image coords
            title=dict(text="v"),
            scaleanchor="x",
        ),
    )

    # TODO(Jack): Why is this in this method??? See comment at top of function.
    # Get the number of frames to fill the max value of the slider
    statistics, _ = data
    n_frames = statistics[sensor]["total_frames"]

    return xy_fig, pixel_fig, max(n_frames - 1, 0)


# TODO(Jack): Are we doing this right at all or should we be using a patch to hold the points and avoiding the json
#  stringify thing?
app.clientside_callback(
    """
    function(frame_idx, sensor, pose_type, cmax, raw_data, processed_data,  xy_fig, pixel_fig) {
        if (!sensor || !pose_type || !raw_data || !processed_data  || !xy_fig || !pixel_fig) {
            console.log("One or more of the inputs is missing.");
            return [dash_clientside.no_update, dash_clientside.no_update];
        }
        
        const timestamps = processed_data[1][sensor]
        if (!timestamps || timestamps.length == 0 || timestamps.length <= frame_idx){
            console.log("Invalid timestamps or frame index out of bounds:", sensor);
            return [dash_clientside.no_update, dash_clientside.no_update];
        }
        
        const timestamp_i = BigInt(timestamps[frame_idx])
        if (!raw_data[sensor] || !raw_data[sensor]['frames'] || !raw_data[sensor]['frames'][timestamp_i]) {
            console.log("Raw data structure is incomplete for sensor:", sensor);
            return [dash_clientside.no_update, dash_clientside.no_update];
        }
        
        const extracted_target = raw_data[sensor]['frames'][timestamp_i].extracted_target
        if (!extracted_target) {
            console.log("No extracted_target found at frame:", frame_idx, "for sensor:", sensor);
            return [dash_clientside.no_update, dash_clientside.no_update];
        }

        const pts = extracted_target.points;
        const xy_patch = new dash_clientside.Patch();
        xy_patch.assign(['data', 0, 'x'], pts.map(p => p[0]));
        xy_patch.assign(['data', 0, 'y'], pts.map(p => p[1]));
        
        const pix = extracted_target.pixels;
        const pixel_patch = new dash_clientside.Patch();
        pixel_patch.assign(['data', 0, 'x'], pix.map(p => p[0]));
        pixel_patch.assign(['data', 0, 'y'], pix.map(p => p[1]));
        
        return [xy_patch.build(), pixel_patch.build()];
    }
    """,
    Output("targets-xy-graph", "figure"),
    Output("targets-pixels-graph", "figure"),
    Input("frame-id-slider", "value"),
    Input("sensor-dropdown", "value"),
    Input("pose-type-selector", "value"),
    Input("max-reprojection-error-input", "value"),
    State("raw-data-store", "data"),
    State("processed-data-store", "data"),
    State("targets-xy-graph", "figure"),
    State("targets-pixels-graph", "figure"),
)
