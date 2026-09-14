import numpy as np
import pandas as pd
from dash import Patch

from business_logic.geometry import InvertSe3

POSE_COLUMNS = ["rx", "ry", "rz", "x", "y", "z"]
IMU_COLUMNS = ["omega_x", "omega_y", "omega_z", "ax", "ay", "az"]


def timeseries_6d_to_patch(rows, *, pose=False, errors=None):
    """Plot SQL rows, aligning IMU errors through their source measurement key."""
    data = pd.DataFrame(rows)
    columns = POSE_COLUMNS if pose else IMU_COLUMNS
    timestamps = []
    values = np.empty((0, 6))
    error_values = None
    if not data.empty:
        data["timestamp_ns"] = data["timestamp_ns"].map(int)
        data = data.sort_values(["timestamp_ns", "step_id"])
        timestamps = data["timestamp_ns"].tolist()
        values = data[columns].to_numpy()
        if pose:
            # The database stores camera-from-world; display camera motion in world.
            values = np.array([InvertSe3(value) for value in values])
        elif errors:
            error_rows = pd.DataFrame(errors)
            error_rows["timestamp_ns"] = error_rows["timestamp_ns"].map(int)
            keys = ["step_id", "asset_id", "timestamp_ns"]
            error_rows = error_rows.drop(columns="step_id").rename(
                columns={"source_step_id": "step_id"}
            )
            aligned = data[keys].merge(
                error_rows, on=keys, how="left", validate="one_to_one"
            )
            error_values = aligned[columns].to_numpy()

    patch = Patch()
    for i in range(6):
        patch["data"][i]["x"] = timestamps
        patch["data"][i]["y"] = values[:, i]
        # Clear errors when switching to a step without them.
        patch["data"][i]["error_y"] = {"visible": False}
        if error_values is not None:
            patch["data"][i]["error_y"] = {
                "color": "black",
                "type": "data",
                "symmetric": False,
                "array": np.maximum(error_values[:, i], 0.0),
                "arrayminus": np.maximum(-error_values[:, i], 0.0),
                "visible": True,
                "thickness": 1,
                "width": 0,
            }
    return patch
