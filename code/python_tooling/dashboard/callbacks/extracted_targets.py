from dash import MATCH, Input, Output, State

from dashboard.server import app
from database.types import SensorType

app.clientside_callback(
    """
    function(frame_idx, targets, step_id, workflow_data, cmax, composite_id) {
        if (!composite_id || !workflow_data) {
            return dash_clientside.no_update;
        }
        const asset_id = composite_id.asset_id;
        const row = (targets || [])[frame_idx];
        if (!row || row.asset_id !== asset_id) {
            const empty = new dash_clientside.Patch();
            for (let i = 0; i < 2; i++) {
                empty.assign(['data', i, 'x'], []);
                empty.assign(['data', i, 'y'], []);
            }
            return empty.build();
        }
        const target = row.data;
        // WARN(Jack): We are hardcoding that the points are 2D here, only taking into account (x,y) while ignoring z.
        const points = target.points.map(row => row.slice(0, 2));
        const pixels = target.pixels.map(row => row.slice(0, 2));
        const indices = target.indices.map(row => row.slice(0, 2))
    
        const patch = new dash_clientside.Patch();
        patch.assign(['data', 0, 'x'], points.map(p => p[0]));
        patch.assign(['data', 0, 'y'], points.map(p => p[1]));
        patch.assign(['data', 1, 'x'], pixels.map(p => p[0]));
        patch.assign(['data', 1, 'y'], pixels.map(p => p[1]));
        patch.assign(['data', 0, 'marker'], {
            size: 12,
            color: "darkgray"
        });
        patch.assign(['data', 1, 'marker'], {
            size: 12,
            color: "darkgray"
        });
        patch.assign(['data', 0, 'customdata'], indices);
        patch.assign(['data', 1, 'customdata'], indices);
    
        const error = ((workflow_data.tables || {}).reprojection_errors || []).find(error =>
            error.asset_id === asset_id && error.step_id === step_id &&
            error.source_step_id === row.step_id && error.sample_timestamp_ns === row.timestamp_ns);
        if (error && cmax > 0) {
            const reprojection_error = error.data;
            patch.assign(['data', 0, 'marker'], {
                size: 12,
                color: reprojection_error.map(p => Math.sqrt(p[0] * p[0] + p[1] * p[1])),
                colorscale: "Bluered",
                cmin: 0,
                cmax: cmax,
            });
            patch.assign(['data', 1, 'marker'], {
                size: 12,
                color: reprojection_error.map(p => Math.sqrt(p[0] * p[0] + p[1] * p[1])),
                colorscale: "Bluered",
                cmin: 0,
                cmax: cmax,
            });
        }
    
        patch.assign(
            ['data', 0, 'hovertemplate'],
            "xy: (%{x:.2f}, %{y:.2f})<br>" +
            "id: (%{customdata[0]}, %{customdata[1]})<br>" +
            "error: %{marker.color:.2f}<extra></extra>"
        );
    
        patch.assign(
            ['data', 1, 'hovertemplate'],
            "uv: (%{x:.2f}, %{y:.2f})<br>" +
            "id: (%{customdata[0]}, %{customdata[1]})<br>" +
            "error: %{marker.color:.2f}<extra></extra>"
        );
    
        // Extracted points give target bounds without inventing a camera/target relationship.
        for (const [axis, coordinate] of [['xaxis', 0], ['yaxis', 1]]) {
            const values = points.map(point => point[coordinate]);
            if (values.length) {
                const low = values.reduce((a, b) => Math.min(a, b));
                const high = values.reduce((a, b) => Math.max(a, b));
                const margin = Math.max((high - low) * 0.05, 0.001);
                patch.assign(['layout', axis, 'range'], [low - margin, high + margin]);
                patch.assign(['layout', axis, 'autorange'], false);
            }
        }
        const camera = ((workflow_data.tables || {}).camera_info || []).find(info => info.asset_id === asset_id);
        if (camera) {
            patch.assign(['layout', 'xaxis2', 'range'], [0, camera.width]);
            patch.assign(['layout', 'yaxis2', 'range'], [0, camera.height]);
            patch.assign(['layout', 'xaxis2', 'autorange'], false);
            patch.assign(['layout', 'yaxis2', 'autorange'], false);
        }

        return patch.build();
    }
    """,
    Output(
        {"type": "extracted_targets", "asset_id": MATCH},
        "figure",
    ),
    Input(
        {"type": "slider", "asset_id": MATCH, "sensor_type": SensorType.Camera},
        "value",
    ),
    Input("selected-targets-store", "data"),
    Input("step-selector", "value"),
    Input("workflow-data-store", "data"),
    Input({"type": "max_error", "asset_id": MATCH}, "value"),
    State({"type": "extracted_targets", "asset_id": MATCH}, "id"),
)
