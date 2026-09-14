from dash import MATCH, Input, Output, no_update

from dashboard.server import app
from dashboard.tools.selection import table_rows
from dashboard.tools.timeseries_6d import timeseries_6d_to_patch
from database.types import SensorType


@app.callback(
    Output({"type": "timeseries", "asset_id": MATCH, "sensor_type": SensorType.Camera}, "figure"),
    Input({"type": "timeseries", "asset_id": MATCH, "sensor_type": SensorType.Camera}, "id"),
    Input("step-selector", "value"),
    Input("workflow-data-store", "data"),
)
def update_camera_timeseries(composite_id, step_id, workflow_data):
    return update_timeseries(composite_id, step_id, workflow_data)


@app.callback(
    Output({"type": "timeseries", "asset_id": MATCH, "sensor_type": SensorType.Imu}, "figure"),
    Input({"type": "imu-step-selector", "asset_id": MATCH}, "id"),
    Input({"type": "imu-step-selector", "asset_id": MATCH}, "value"),
    Input("workflow-data-store", "data"),
)
def update_imu_timeseries(composite_id, step_id, workflow_data):
    if composite_id is None:
        return no_update
    return update_timeseries(
        dict(composite_id, sensor_type=SensorType.Imu), step_id, workflow_data
    )


def update_timeseries(composite_id, step_id, workflow_data):
    if composite_id is None or workflow_data is None:
        return no_update

    asset_id = composite_id["asset_id"]
    sensor_type = composite_id["sensor_type"]

    if sensor_type == SensorType.Camera:
        rows = (
            table_rows(
                workflow_data, "camera_poses", asset_id=asset_id, step_id=step_id
            )
            if step_id is not None
            else []
        )
        return timeseries_6d_to_patch(rows, pose=True)
    if sensor_type == SensorType.Imu:
        rows = table_rows(workflow_data, "imu_data", asset_id=asset_id)
        errors = (
            table_rows(workflow_data, "imu_errors", asset_id=asset_id, step_id=step_id)
            if step_id is not None
            else []
        )
        if errors:
            sources = {error["source_step_id"] for error in errors}
            rows = [row for row in rows if row["step_id"] in sources]
        else:
            selected = [row for row in rows if row["step_id"] == step_id]
            rows = selected or rows
        return timeseries_6d_to_patch(rows, errors=errors)
    return no_update


app.clientside_callback(
    """
    function(timestamp_ns) {
        if (timestamp_ns == null || timestamp_ns === "") {
            const patch = new dash_clientside.Patch();
            patch.assign(['layout', 'shapes'], []);
            return patch.build();
        }
    
        const timestamp_ns_int = Number(BigInt(timestamp_ns));
    
        const new_shape = {
            type: 'rect',
            xref: 'x',
            yref: 'paper',
            x0: timestamp_ns_int,
            x1: timestamp_ns_int,
            y0: 0,
            y1: 1,
            line: {
                color: 'black',
                width: 1
            },
        };
    
        const patch = new dash_clientside.Patch();
        patch.assign(['layout', 'shapes'], [new_shape]);
    
        return patch.build();
    }
    """,
    Output(
        {"type": "timeseries", "asset_id": MATCH, "sensor_type": SensorType.Camera},
        "figure",
        allow_duplicate=True,
    ),
    Input(
        {
            "type": "current_timestamp",
            "asset_id": MATCH,
            "sensor_type": SensorType.Camera,
        },
        "children",
    ),
    prevent_initial_call=True,
)
