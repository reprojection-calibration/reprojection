import plotly.graph_objects as go


def measurement_delta_time_figures(df, label="Measurement"):
    assert not df.empty, "df is empty"
    assert "sensor_name" in df.columns, f"Missing column: sensor_name"
    assert "timestamp_ns" in df.columns, f"Missing column: timestamp_ns"

    assert df["sensor_name"].nunique() == 1, (
        f"Expected exactly one sensor_name, "
        f"found: {df['sensor_name'].unique().tolist()}"
    )

    rows = df.sort_values("timestamp_ns").copy()

    rows["time_s"] = (rows["timestamp_ns"] - rows["timestamp_ns"].iloc[0]) * 1e-9
    rows["delta_ms"] = rows["timestamp_ns"].diff() * 1e-6
    rows = rows.dropna(subset=["delta_ms"])

    assert not rows.empty, "Need at least two timestamps to compute deltas"

    delta_fig = go.Figure()
    delta_fig.add_trace(
        go.Scattergl(
            x=rows["time_s"],
            y=rows["delta_ms"],
            mode="markers",
            marker=dict(size=4, opacity=0.7),
        )
    )
    delta_fig.update_layout(
        title=f"{label} measurement delta time",
        xaxis=dict(title="Measurement time [s]"),
        yaxis=dict(title="Delta time [ms]"),
        showlegend=False,
    )

    histogram_fig = go.Figure()
    histogram_fig.add_trace(
        go.Histogram(
            x=rows["delta_ms"],
            nbinsx=100,
        )
    )
    histogram_fig.update_layout(
        title=f"{label} measurement interval histogram",
        xaxis=dict(title="Delta time [ms]"),
        yaxis=dict(title="Frequency"),
        showlegend=False,
    )

    return delta_fig, histogram_fig
