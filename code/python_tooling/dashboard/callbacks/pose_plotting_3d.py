import plotly.graph_objects as go
from dash import Input, Output, State

from dashboard.server import app


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
# TODO(Jack): Do we really require pose_fig_3d as input?
app.clientside_callback(
    """
    function(frame_idx, sensor, pose_type, raw_data, metadata, pose_fig_3d) {
        if (!sensor || !pose_type || !raw_data || !metadata || !pose_fig_3d) {
            return dash_clientside.no_update;
        }
    
        const timestamps = window.dataInputUtils.getTimestamps(metadata, "camera", sensor);
        if (!timestamps) {
            return dash_clientside.no_update;
        }
        
        const frame_result = window.dataInputUtils.getValidFrame(
            raw_data, sensor, timestamps, frame_idx
        );
        if (!frame_result) {
            return dash_clientside.no_update;
        }
        const { frame, _ } = frame_result;
    
        // ERROR(Jack): We need to protect against poses or pose_type not being available
        const pose = frame['poses'][pose_type]
        if (!pose) {
            return dash_clientside.no_update;
        }
    
        const [rx, ry, rz, x, y, z] = pose;


        const theta = Math.sqrt(rx*rx + ry*ry + rz*rz);

        let R = [
            [1,0,0],
            [0,1,0],
            [0,0,1]
        ];
        if (theta > 1e-8) {
            const kx = rx/theta;
            const ky = ry/theta;
            const kz = rz/theta;

            const c = Math.cos(theta);
            const s = Math.sin(theta);
            const v = 1 - c;

            R = [
                [kx*kx*v + c,     kx*ky*v - kz*s, kx*kz*v + ky*s],
                [ky*kx*v + kz*s,  ky*ky*v + c,    ky*kz*v - kx*s],
                [kz*kx*v - ky*s,  kz*ky*v + kx*s, kz*kz*v + c]
            ];
        }

        const scale = 0.5;
        const origin = [x, y, z];
        const x_axis = [
            x + scale * R[0][0],
            y + scale * R[1][0],
            z + scale * R[2][0]
        ];
        const y_axis = [
            x + scale * R[0][1],
            y + scale * R[1][1],
            z + scale * R[2][1]
        ];
        const z_axis = [
            x + scale * R[0][2],
            y + scale * R[1][2],
            z + scale * R[2][2]
        ];
        

    
        
        const patch = new dash_clientside.Patch();

        patch.assign(['data', 0, 'x'], [origin[0], x_axis[0]]);
        patch.assign(['data', 0, 'y'], [origin[1], x_axis[1]]);
        patch.assign(['data', 0, 'z'], [origin[2], x_axis[2]]);

        patch.assign(['data', 1, 'x'], [origin[0], y_axis[0]]);
        patch.assign(['data', 1, 'y'], [origin[1], y_axis[1]]);
        patch.assign(['data', 1, 'z'], [origin[2], y_axis[2]]);

        patch.assign(['data', 2, 'x'], [origin[0], z_axis[0]]);
        patch.assign(['data', 2, 'y'], [origin[1], z_axis[1]]);
        patch.assign(['data', 2, 'z'], [origin[2], z_axis[2]]);

        return patch.build();
    }
    """,
    Output("camera-3d-pose-graph", "figure"),
    Input("camera-frame-id-slider", "value"),
    Input("camera-sensor-dropdown", "value"),
    Input("pose-type-selector", "value"),
    State("raw-camera-data-store", "data"),
    State("metadata-store", "data"),
    State("camera-3d-pose-graph", "figure"),
)
