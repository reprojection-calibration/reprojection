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
        # Convert the timestamp that are the error dict keys to int (invert the json serialization effect).
        error_by_timestamp = {
            int(timestamp): value for timestamp, value in error.items()
        }

        # Insert nans at any timestamps which does not exist in the errors. This makes sure the errors stays
        # correspondent with the underlying data.
        error_array = np.array(
            [
                error_by_timestamp.get(timestamp, [np.nan] * 6)
                for timestamp in timestamps
            ]
        )

        def add_error_trace(data, id, patch):
            patch["data"][id]["marker"]["color"] = data[:, i]
            patch["data"][id]["marker"]["colorscale"] = "RdBu"
            # TODO(Jack): We should be able to set these ranges from the GUI
            patch["data"][id]["marker"]["cmin"] = -1
            patch["data"][id]["marker"]["cmax"] = 1

            # We only need to plot one scale bar
            if id == 0:
                patch["data"][id]["marker"]["showscale"] = True
                patch["data"][id]["marker"]["colorbar"] = {
                    "title": {"text": "Signed Error"},
                    "x": 1.1,
                    "y": 0.5,
                    "len": 1,
                }

        for i in range(6):
            add_error_trace(error_array, i, patch)

    return patch
