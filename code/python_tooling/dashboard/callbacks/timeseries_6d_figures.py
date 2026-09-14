from dash import MATCH, Input, Output, no_update

from dashboard.server import app
from dashboard.tools.selection import table_rows
from dashboard.tools.timeseries_6d import timeseries_6d_to_patch
from database.types import SensorType


# TODO(Jack): That fact that we have two inputs here means that when we switch between sensors we get two calls here,
#  once when the step-selector gets triggered and once when we the composite_id arrives once the dynamic layout has been
#  updated. This seems like we are missing some abstraction here.
@app.callback(
    Output({"type": "timeseries", "asset_id": MATCH, "sensor_type": MATCH}, "figure"),
    Input({"type": "timeseries", "asset_id": MATCH, "sensor_type": MATCH}, "id"),
    Input("step-selector", "value"),
    Input("workflow-data-store", "data"),
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
