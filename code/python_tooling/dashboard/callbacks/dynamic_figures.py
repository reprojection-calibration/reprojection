import numpy as np
from dash import MATCH, Input, Output, Patch, State, no_update

from dashboard.server import app


# TODO(Jack): Add ability so select step_name
@app.callback(
    Output({"type": "camera-figure", "sensor": MATCH, "subtype": "poses"}, "figure"),
    Input({"type": "camera-figure", "sensor": MATCH, "subtype": "poses"}, "id"),
    State("raw-data-store", "data"),
)
def update_camera_poses(composite_id, raw_data):
    if raw_data is None:
        return no_update

    sensor_name = composite_id["sensor"]
    # TODO(Jack): Harden against non existent data
    pose_data = raw_data[sensor_name]["poses"]["linear_pose_initialization"]

    # TODO(Jack): Why do we have the stimestamps data store at all?
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

    return patch
