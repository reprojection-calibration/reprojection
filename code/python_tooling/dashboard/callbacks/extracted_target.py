import plotly.graph_objects as go
from dash import Input, Output, State

from dashboard.server import IMAGE_DIMENSIONS, app


@app.callback(
    Output("targets-xy-graph", "figure", allow_duplicate=True),
    Output("targets-pixels-graph", "figure", allow_duplicate=True),
    Input("camera-sensor-dropdown", "value"),
    prevent_initial_call=True,
)
def init_extracted_target_figures(sensor):
    if not sensor:
        return {}, {}

    # TODO(Jack): Confirm/test ALL axes properties (ranges, names, orders etc.) None of this has been checked! Even the
    #  coordinate conventions of the pixels and points needs to be checked!
    # TODO(Jack): Eliminate copy and paste here in this method! We basically do the same thing twice.
    # TODO(Jack): We have now copy and pasted in several places that the marker size for xy_fig is 12 and for pixel_fig
    #  is 6. This is a hack! We need to auto scale all dimensions and all marker sizes! Or at least make them more
    #  generic.
    xy_fig = go.Figure()
    xy_fig.add_trace(
        go.Scattergl(
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
        go.Scattergl(
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

    return xy_fig, pixel_fig


# NOTE(Jack): Manually formatted periodically using https://beautifier.io/ - we should automate this process!
app.clientside_callback(
    """
    function(frame_idx, sensor, pose_type, cmax, raw_data, metadata, xy_fig, pixel_fig) {
        if (!sensor || !pose_type || !raw_data || !metadata || !xy_fig || !pixel_fig) {
            return [dash_clientside.no_update, dash_clientside.no_update];
        }
    
        // TODO(Jack): Do we need to protect against "camera" being available here, or can we take that for granted?
        const timestamps = metadata[1]["camera"][sensor]
        if (!timestamps || timestamps.length == 0 || timestamps.length <= frame_idx) {
            return [dash_clientside.no_update, dash_clientside.no_update];
        }
    
        const timestamp_i = BigInt(timestamps[frame_idx])
        if (!raw_data[sensor] || !raw_data[sensor]['frames'] || !raw_data[sensor]['frames'][timestamp_i]) {
            return [dash_clientside.no_update, dash_clientside.no_update];
        }
    
        const extracted_target = raw_data[sensor]['frames'][timestamp_i].extracted_target
        if (!extracted_target) {
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
    
        // If reprojection errors are available we will color the points and pixels according to them. If not available
        // simply return the figures with the plain colored points and pixels.
        const reprojection_errors = raw_data[sensor]['frames'][timestamp_i]['reprojection_errors']
        if (!reprojection_errors) {
            // If no reprojection errors are available at all then return to default marker configuration.
            xy_patch.assign(['data', 0, 'marker'], {
                size: 12
            });
            pixel_patch.assign(['data', 0, 'marker'], {
                size: 6
            });
    
            return [xy_patch.build(), pixel_patch.build()];
        }
    
        const reprojection_error = reprojection_errors[pose_type]
        if (!reprojection_error) {
            // If the sensor specific reprojection error is not available then return to default marker configuration.
            xy_patch.assign(['data', 0, 'marker'], {
                size: 12
            });
            pixel_patch.assign(['data', 0, 'marker'], {
                size: 6
            });
    
            return [xy_patch.build(), pixel_patch.build()];
        }
    
        xy_patch.assign(['data', 0, 'marker'], {
            size: 12,
            color: reprojection_error.map(p => Math.sqrt(p[0] * p[0] + p[1] * p[1])),
            colorscale: "Bluered",
            cmin: 0,
            cmax: cmax,
            showscale: true
        });
        pixel_patch.assign(['data', 0, 'marker'], {
            size: 6,
            color: reprojection_error.map(p => Math.sqrt(p[0] * p[0] + p[1] * p[1])),
            colorscale: "Bluered",
            cmin: 0,
            cmax: cmax,
            showscale: true
        });
    
        return [xy_patch.build(), pixel_patch.build()];
    }
    """,
    Output("targets-xy-graph", "figure"),
    Output("targets-pixels-graph", "figure"),
    Input("camera-frame-id-slider", "value"),
    Input("camera-sensor-dropdown", "value"),
    Input("pose-type-selector", "value"),
    Input("max-reprojection-error-input", "value"),
    State("raw-camera-data-store", "data"),
    State("metadata-store", "data"),
    State("targets-xy-graph", "figure"),
    State("targets-pixels-graph", "figure"),
)
