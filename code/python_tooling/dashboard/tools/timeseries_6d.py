import numpy as np
from dash import Patch


def timeseries_6d_to_patch(data, error):
    # NOTE(Jack): Json serialization will automatically convert the dictionary timestamp keys to string. Therefore we
    # need to convert them back to int here.
    timestamps = list(data.keys())
    timestamps = [int(t) for t in timestamps]
    array = np.array(list(data.values()))

    def add_trace(timestamps, data, id, patch):
        patch["data"][id]["x"] = timestamps
        patch["data"][id]["y"] = data[:, id]

    patch = Patch()
    for i in range(6):
        add_trace(timestamps, array, i, patch)

    # TODO(Jack): Consider doing a relative error instead of an absolute error. Cause right now when the thing is moving
    #  slowly it looks ok but it might be a large relative error.
    if error is not None:
        error_by_timestamp = {
            int(timestamp): value for timestamp, value in error.items()
        }

        error_array = np.array(
            [
                error_by_timestamp.get(timestamp, [np.nan] * 6)
                for timestamp in timestamps
            ]
        )

        for i in range(6):
            patch["data"][i]["error_y"] = {
                "color": "black",
                "type": "data",
                "symmetric": False,
                "array": np.maximum(error_array[:, i], 0.0),
                "arrayminus": np.maximum(-error_array[:, i], 0.0),
                "visible": True,
                "thickness": 1,
                "width": 0,
            }

    return patch
