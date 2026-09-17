from dash import ALL, MATCH, Input, Output, State

from dashboard.server import app
from database.types import SensorType

app.clientside_callback(
    """
    function(frame_indices, targets, step_id, workflow_data, cmax_values, stage_id, composite_id) {
        if (!composite_id || !workflow_data) {
            return dash_clientside.no_update;
        }
        // There is one shared slider and colour scale in either layout.
        const frame_idx = frame_indices[0] || 0;
        const cmax = cmax_values[0];
        const asset_id = composite_id.asset_id;
        const multi = stage_id === 'multi_cam' || stage_id === 'cam_imu';
        const frame = (targets || [])[frame_idx];
        const row = multi ? (frame?.cameras || {})[asset_id] : frame;
        const patch = new dash_clientside.Patch();
        const feature_trace = multi ? 0 : 1;
        const feature_axis = multi ? '' : '2';
        const camera = ((workflow_data.tables || {}).camera_info || []).find(info => info.asset_id === asset_id);
        if (camera) {
            patch.assign(['layout', 'xaxis' + feature_axis, 'range'], [0, camera.width]);
            patch.assign(['layout', 'yaxis' + feature_axis, 'range'], [0, camera.height]);
            patch.assign(['layout', 'xaxis' + feature_axis, 'autorange'], false);
            patch.assign(['layout', 'yaxis' + feature_axis, 'autorange'], false);
        }
        if (!row || row.asset_id !== asset_id) {
            for (let i = 0; i <= feature_trace; i++) {
                for (const field of ['x', 'y', 'customdata']) {
                    patch.assign(['data', i, field], []);
                }
                patch.assign(['data', i, 'marker'], {size: 12, color: 'darkgray'});
            }
            if (multi) {
                patch.assign(['layout', 'title', 'text'], 'Extracted features · No matching sample');
            }
            return patch.build();
        }
        const target = row.data;
        const error = ((workflow_data.tables || {}).reprojection_errors || []).find(error =>
            error.asset_id === asset_id && error.step_id === step_id &&
            error.source_step_id === row.step_id && error.sample_timestamp_ns === row.timestamp_ns);
        const marker = {size: 12, color: 'darkgray'};
        if (error && cmax > 0) {
            Object.assign(marker, {
                color: error.data.map(p => Math.hypot(p[0], p[1])),
                colorscale: 'Bluered', cmin: 0, cmax: cmax,
            });
        }
        function plot(trace, points, coordinates) {
            patch.assign(['data', trace, 'x'], points.map(p => p[0]));
            patch.assign(['data', trace, 'y'], points.map(p => p[1]));
            patch.assign(['data', trace, 'customdata'], target.indices);
            patch.assign(['data', trace, 'marker'], marker);
            patch.assign(['data', trace, 'hovertemplate'],
                coordinates + ': (%{x:.2f}, %{y:.2f})<br>' +
                'id: (%{customdata[0]}, %{customdata[1]})<br>' +
                'error: %{marker.color:.2f}<extra></extra>');
        }
        plot(feature_trace, target.pixels, 'uv');
        if (multi) {
            // Subtract integers before conversion: epoch nanoseconds exceed JS number precision.
            const delta_ns = BigInt(row.timestamp_ns) - BigInt(frame.timestamp_ns);
            const delta_ms = Number(delta_ns) / 1e6;
            const delta = (delta_ns >= 0n ? '+' : '') + delta_ms.toFixed(6);
            patch.assign(['layout', 'title', 'text'],
                'Extracted features · Δt (' + frame.time_basis + ') = ' + delta + ' ms');
        } else {
            plot(0, target.points, 'xy');
            for (const [axis, coordinate] of [['xaxis', 0], ['yaxis', 1]]) {
                const values = target.points.map(point => point[coordinate]);
                if (values.length) {
                    const low = values.reduce((a, b) => Math.min(a, b));
                    const high = values.reduce((a, b) => Math.max(a, b));
                    const margin = Math.max((high - low) * 0.05, 0.001);
                    patch.assign(['layout', axis, 'range'], [low - margin, high + margin]);
                    patch.assign(['layout', axis, 'autorange'], false);
                }
            }
        }
        return patch.build();
    }
    """,
    Output({"type": "extracted_targets", "asset_id": MATCH}, "figure"),
    Input(
        {"type": "slider", "asset_id": ALL, "sensor_type": SensorType.Camera}, "value"
    ),
    Input("selected-targets-store", "data"),
    Input("step-selector", "value"),
    Input("workflow-data-store", "data"),
    Input({"type": "max_error", "asset_id": ALL}, "value"),
    Input("stage-selector", "value"),
    State({"type": "extracted_targets", "asset_id": MATCH}, "id"),
)
