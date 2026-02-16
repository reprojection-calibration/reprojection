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
    pose_graph_3d = go.Figure()

    # X axis trace (trace 0)
    pose_graph_3d.add_trace(
        go.Scatter3d(
            x=[0, 1],
            y=[0, 0],
            z=[0, 0],
            line=dict(
                color="red",
                width=6,
            ),
            mode="lines",
            name="X axis",
        )
    )

    # Y axis trace (trace 1)
    pose_graph_3d.add_trace(
        go.Scatter3d(
            x=[0, 0],
            y=[0, 1],
            z=[0, 0],
            line=dict(
                color="green",
                width=6,
            ),
            mode="lines",
            name="Y axis",
        )
    )

    # Z axis trace (trace 2)
    pose_graph_3d.add_trace(
        go.Scatter3d(
            x=[0, 0],
            y=[0, 0],
            z=[0, 1],
            line=dict(
                color="blue",
                width=6,
            ),
            mode="lines",
            name="Z axis",
        )
    )

    pose_graph_3d.update_layout(
        title="3D Sensor Pose",
        scene=dict(
            aspectmode="cube",
            xaxis=dict(range=[-1, 1], autorange=False),
            yaxis=dict(range=[-1, 1], autorange=False),
            zaxis=dict(range=[-1, 1], autorange=False),
            camera=dict(
                eye=dict(x=1, y=1, z=2),
                up=dict(x=0, y=1, z=0),
            ),
        ),
        margin=dict(l=0, r=0, t=40, b=0),
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
