import numpy as np
from dash import Patch


def timeseries_6d_to_patch(data):
    # NOTE(Jack): Json serialization will automatically convert the dictionary timestamp keys to string. Therefore we
    # need to convert them back to int here.
    timestamps = list(data.keys())
    timestamps = [int(t) for t in timestamps]
    array = np.array(list(data.values()))

    def add_trace(timestamps, data, id, patch):
        patch["data"][id]["x"] = timestamps
        patch["data"][id]["y"] = data[:, id]

    patch = Patch()
    for i in range(5):
        add_trace(timestamps, array, i, patch)

    return patch
