from dash import Input, Output,  State

from dashboard.server import app
import plotly.graph_objects as go


# TODO(Jack): Refactor to use register pattern
@app.callback(
    Output("camera-3d-pose-graph", "figure", allow_duplicate=True),
    Input("camera-sensor-dropdown", "value"),
    prevent_initial_call=True,
)
def build_3d_pose_graph_callback(sensor):
    if not sensor:
        return {}

    # NOTE(Jack): A number cannot be the first letter in a variable name
    pose_graph_3d = go.Figure()
    pose_graph_3d.add_trace(
        go.Scatter3d(
            x=[1],
            y=[1],
            z=[1],
        )
    )

    pose_graph_3d.update_layout(
        title="3D Sensor Pose",
        scene=dict(
            aspectmode="cube",
        ),
    )

    return pose_graph_3d


# TODO(Jack): Do not hardcode const timestamps = metadata[1]["camera"][sensor]
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
    
        const extracted_target = raw_data[sensor]['frames']['poses'][pose_type]
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