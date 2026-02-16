import plotly.graph_objects as go
from dash import Input, Output, State

from dashboard.server import app


# TODO(Jack): Refactor to use register pattern
@app.callback(
    Output("camera-3d-pose-graph", "figure", allow_duplicate=True),
    Input("camera-sensor-dropdown", "value"),
    prevent_initial_call=True,
)
def build_3d_pose_graph_callback(_):

    def add_trace(color, pose_graph_3d):
        pose_graph_3d.add_trace(
            go.Scatter3d(
                line=dict(
                    color=color,
                    width=6,
                ),
                mode="lines",
            )
        )

    pose_graph_3d = go.Figure()
    add_trace("red", pose_graph_3d)  # trace 0, x-axis
    add_trace("green", pose_graph_3d)  # trace 1, y-axis
    add_trace("blue", pose_graph_3d)  # trace 2, z-axis

    pose_graph_3d.update_layout(
        showlegend=False,
        title="3D Sensor Pose",
        scene=dict(
            aspectmode="cube",
            # TODO(Jack): Do not hardcode these ranges! Should be set from calculated metadata? Same with view point.
            xaxis=dict(range=[-1, 1], autorange=False),
            yaxis=dict(range=[-1, 1], autorange=False),
            zaxis=dict(range=[-1, 1], autorange=False),
            # View to be looking at the target from over and behind the camera - "down the barrel" view
            camera=dict(
                eye=dict(x=1, y=1, z=2),
                up=dict(x=0, y=1, z=0),
            ),
        ),
    )

    return pose_graph_3d


# TODO(Jack): Do not hardcode const timestamps = metadata[1]["camera"][sensor]
app.clientside_callback(
    """
    function(frame_idx, sensor, pose_type, raw_data, metadata) {
        if (frame_idx == null || !sensor || !pose_type || !raw_data || !metadata) {
            return dash_clientside.no_update;
        }
    
        // DO NOT HARDCODE CAMERA
        const timestamp_result = window.dataInputUtils.getTimestamps(metadata, "camera", sensor, frame_idx);
        if (!timestamp_result) {
            return dash_clientside.no_update;
        }
        const {_, timestamp_i} = timestamp_result;
        
        const frame = window.dataInputUtils.getValidFrame(
            raw_data, sensor, timestamp_i
        );
        if (!frame) {
            return dash_clientside.no_update;
        }
    
        const pose = frame?.poses?.[pose_type];
            if (!pose) {
            return dash_clientside.no_update;
        }

        const R = window.tfUtils.toRotationMatrix(pose.slice(0,3));

        const origin = pose.slice(3,6)
        const scale = 0.5;
        const x_axis = window.tfUtils.buildAxisVector(origin, R, "x", scale);
        const y_axis = window.tfUtils.buildAxisVector(origin, R, "y", scale);
        const z_axis = window.tfUtils.buildAxisVector(origin, R, "z", scale);
        
        patch = new dash_clientside.Patch();
        window.tfUtils.addAxisVector(origin, x_axis, "x", patch);
        window.tfUtils.addAxisVector(origin, y_axis, "y", patch);
        window.tfUtils.addAxisVector(origin, z_axis, "z", patch);

        return patch.build();
    }
    """,
    Output("camera-3d-pose-graph", "figure"),
    Input("camera-frame-id-slider", "value"),
    Input("camera-sensor-dropdown", "value"),
    Input("pose-type-selector", "value"),
    State("raw-camera-data-store", "data"),
    State("metadata-store", "data"),
)
