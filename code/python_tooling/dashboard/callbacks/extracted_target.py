import plotly.graph_objects as go
from dash import Input, Output, State

from dashboard.server import IMAGE_DIMENSIONS, app


@app.callback(
    Output("targets-xy-graph", "figure", allow_duplicate=True),
    Output("targets-pixels-graph", "figure", allow_duplicate=True),
    Input("camera-sensor-dropdown", "value"),
    prevent_initial_call=True,
)
def build_extracted_target_figures_callback(_):
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
        # TODO(Jack): Is inverting the y really necessary here? Does that do what we want?
        yaxis=dict(
            range=[IMAGE_DIMENSIONS[1], 0],
            title=dict(text="v"),
            scaleanchor="x",
        ),
    )

    return xy_fig, pixel_fig


# NOTE(Jack): Manually formatted periodically using https://beautifier.io/ - we should automate this process!
app.clientside_callback(
    """
    function(frame_idx, sensor, pose_type, cmax, raw_data, metadata, xy_fig, pixel_fig) {
        if (frame_idx == null || !sensor || !pose_type || cmax == null || !raw_data || !metadata || !xy_fig || !pixel_fig) {
            return [dash_clientside.no_update, dash_clientside.no_update];
        }
    
        const timestamp_result = window.dataInputUtils.getTimestamps(metadata, "camera", sensor, frame_idx);
        if (!timestamp_result) {
            return [dash_clientside.no_update, dash_clientside.no_update];
        }
        const {_, timestamp_i} = timestamp_result;
        
        const frame = window.dataInputUtils.getValidFrame(
            raw_data, sensor, timestamp_i
        );
        if (!frame) {
            return [dash_clientside.no_update, dash_clientside.no_update];
        }
    
        const extracted_target = frame.extracted_target
        if (!extracted_target) {
            return [dash_clientside.no_update, dash_clientside.no_update];
        }
        
        const reprojection_error = frame?.reprojection_errors?.[pose_type];
    
        // NOTE(Jack): We just ignore the z-dimension and assume its zero for the 3d target points. There might be a day 
        // where this is not a valid assumption (ex. non-flat multi-target configurations)!
        const pts = extracted_target.points.map(row => row.slice(0, 2));
        const xy_patch =  window.extractedTargetUtils.buildExtractedTargetPatch(
            pts, reprojection_error, cmax
        );
        
        const pix = extracted_target.pixels;
        const pixel_patch =  window.extractedTargetUtils.buildExtractedTargetPatch(
            pix, reprojection_error, cmax
        );
    
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
