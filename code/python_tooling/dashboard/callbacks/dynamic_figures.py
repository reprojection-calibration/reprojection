import numpy as np
from dash import MATCH, Input, Output, Patch, State, no_update

from dashboard.server import app


# TODO(Jack): This is not really specific to camera poses, we can rewrite this method to be generic to the sensor type.
# TODO(Jack): That fact that we have two inputs here means that when we swith between sensors we get two calls here,
#  once when the step-selector gets triggered and once when we the composite_id arrives once the dynamic layout has been
#  updated. This seems like we are missing some abstraction here.
@app.callback(
    Output({"type": "camera-figure", "sensor": MATCH, "subtype": "poses"}, "figure"),
    Input({"type": "camera-figure", "sensor": MATCH, "subtype": "poses"}, "id"),
    Input('step-selector', 'value'),
    State("raw-data-store", "data")
)
def update_camera_poses(composite_id, step_name, raw_data):
    if composite_id is None or step_name is None or raw_data is None:
        return no_update

    sensor_name = composite_id["sensor"]
    try:
        pose_data = raw_data[sensor_name]["poses"][step_name]
    except (KeyError, TypeError):
        return no_update

    # TODO(Jack): Why do we have the timestamps data store at all if we do not use it here?
    # NOTE(Jack): Json serialization will automatically convert the dictionary timestamp keys to string. Therefore we
    # need to convert them back to int here.
    timestamps = list(pose_data.keys())
    timestamps = [int(t) for t in timestamps]
    array = np.array(list(pose_data.values()))

    # TODO(Jack): Make method and also plot translation
    patch = Patch()
    patch["data"][0]["x"] = timestamps
    patch["data"][0]["y"] = array[:, 0]
    patch["data"][1]["x"] = timestamps
    patch["data"][1]["y"] = array[:, 1]
    patch["data"][2]["x"] = timestamps
    patch["data"][2]["y"] = array[:, 2]

    patch["data"][3]["x"] = timestamps
    patch["data"][3]["y"] = array[:, 3]
    patch["data"][4]["x"] = timestamps
    patch["data"][4]["y"] = array[:, 4]
    patch["data"][5]["x"] = timestamps
    patch["data"][5]["y"] = array[:, 5]

    return patch
