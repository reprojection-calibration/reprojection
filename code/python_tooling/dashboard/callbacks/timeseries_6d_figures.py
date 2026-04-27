from dash import MATCH, Input, Output, State, no_update

from dashboard.server import app
from dashboard.tools.timeseries_6d import timeseries_6d_to_patch
from database.types import SensorType


# TODO(Jack): That fact that we have two inputs here means that when we switch between sensors we get two calls here,
#  once when the step-selector gets triggered and once when we the composite_id arrives once the dynamic layout has been
#  updated. This seems like we are missing some abstraction here.
@app.callback(
    Output(
        {"type": "timeseries", "sensor_name": MATCH, "sensor_type": MATCH}, "figure"
    ),
    Input({"type": "timeseries", "sensor_name": MATCH, "sensor_type": MATCH}, "id"),
    Input("step-selector", "value"),
    State("raw-data-store", "data"),
)
def update_timeseries(composite_id, step_name, raw_data):
    if composite_id is None or raw_data is None:
        return no_update

    sensor_name = composite_id["sensor_name"]
    sensor_type = composite_id["sensor_type"]

    if sensor_type == SensorType.Camera:
        try:
            data = raw_data[sensor_name]["poses"][step_name]
        except (KeyError, TypeError):
            return no_update
    elif sensor_type == SensorType.Imu:
        try:
            data = raw_data[sensor_name]["measurements"]
        except (KeyError, TypeError):
            return no_update

    return timeseries_6d_to_patch(data)


app.clientside_callback(
    """
    function(timestamp_ns) {
        console.log(console.log(timestamp_ns));
        
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
    
        patch = new dash_clientside.Patch();
        patch.assign(['layout', 'shapes'], [new_shape]);
    
        return patch.build();
    }
    """,
    Output(
        {"type": "timeseries", "sensor_name": MATCH, "sensor_type": MATCH}, "figure", allow_duplicate=True,
    ),
    Input(
        {
            "type": "current_timestamp",
            "sensor_name": MATCH,
            "sensor_type": MATCH,
        },
        "children",
    ),
    prevent_initial_call=True,
)
