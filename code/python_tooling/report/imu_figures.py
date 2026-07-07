import plotly.graph_objects as go


# TODO(Jack): We should also plot this for the camera!
def imu_delta_time_figure(imu_df):
    assert not imu_df.empty, "imu_df is empty"
    assert (
            imu_df["sensor_name"].nunique() == 1
    ), f"Expected exactly one sensor_name, found: {imu_df['sensor_name'].unique().tolist()}"

    rows = imu_df.sort_values("timestamp_ns").copy()

    rows["time_s"] = (rows["timestamp_ns"] - rows["timestamp_ns"].iloc[0]) * 1e-9
    rows["delta_ms"] = rows["timestamp_ns"].diff() * 1e-6

    # The first row has no previous measurement, so delta is NaN.
    rows = rows.dropna(subset=["delta_ms"])

    fig = go.Figure()

    fig.add_trace(
        go.Scattergl(
            x=rows["time_s"],
            y=rows["delta_ms"],
            mode="markers",
            marker=dict(size=4, opacity=0.7),
        )
    )

    fig.update_layout(
        title="IMU measurement delta time",
        xaxis=dict(title="Measurement time [s]"),
        yaxis=dict(title="Delta time [ms]"),
        showlegend=False,
    )

    return fig