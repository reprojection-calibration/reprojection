from dash import MATCH, Input, Output, State, no_update

from dashboard.server import app
from dashboard.tools.timeseries_6d import timeseries_6d_to_patch


# TODO(Jack): This is not really specific to camera poses, we can rewrite this method to be generic to the sensor type.
# TODO(Jack): That fact that we have two inputs here means that when we swith between sensors we get two calls here,
#  once when the step-selector gets triggered and once when we the composite_id arrives once the dynamic layout has been
#  updated. This seems like we are missing some abstraction here.
@app.callback(
    Output({"type": "pose", "sensor": MATCH}, "figure"),
    Input({"type": "pose", "sensor": MATCH}, "id"),
    Input("step-selector", "value"),
    State("raw-data-store", "data"),
)
def update_pose_figure(composite_id, step_name, raw_data):
    if composite_id is None or step_name is None or raw_data is None:
        return no_update

    sensor_name = composite_id["sensor"]
    try:
        pose_data = raw_data[sensor_name]["poses"][step_name]
    except (KeyError, TypeError):
        return no_update

    return timeseries_6d_to_patch(pose_data)

@app.callback(
    Output({"type": "imu_data", "sensor": MATCH}, "figure"),
    Input({"type": "imu_data", "sensor": MATCH}, "id"),
    State("raw-data-store", "data"),
)
def update_imu_data_figure(composite_id, raw_data):
    if composite_id is None or raw_data is None:
        return no_update

    sensor_name = composite_id["sensor"]
    try:
        pose_data = raw_data[sensor_name]["measurements"]
    except (KeyError, TypeError):
        return no_update

    return timeseries_6d_to_patch(pose_data)