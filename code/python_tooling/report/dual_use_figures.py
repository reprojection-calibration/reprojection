import plotly.graph_objects as go


def median_absolute_deviation(values):
    if values.empty:
        return 0

    median = values.median()

    return (values - median).abs().median()


def format_stats(name, values):
    if values.empty:
        return f"{name}: N/A"

    median = values.median()
    mad = median_absolute_deviation(values)

    return f"{name}: median={median:.3f} ms, median absolute deviation={mad:.3f} ms"


def measurement_delta_time_figures(df, label="Measurement"):
    assert not df.empty, "df is empty"
    assert "sensor_name" in df.columns, "Missing column: sensor_name"
    assert "timestamp_ns" in df.columns, "Missing column: timestamp_ns"

    assert df["sensor_name"].nunique() == 1, (
        f"Expected exactly one sensor_name, "
        f"found: {df['sensor_name'].unique().tolist()}"
    )

    rows = df.sort_values("timestamp_ns").copy()

    rows["time_s"] = (rows["timestamp_ns"] - rows["timestamp_ns"].iloc[0]) * 1e-9
    rows["delta_ms"] = rows["timestamp_ns"].diff() * 1e-6

    # The first measurement has no previous value to calculate the time delta with which results in a NaN. This removes
    # that value.
    rows = rows.dropna(subset=["delta_ms"])

    delta_ms = rows["delta_ms"]
    mad = median_absolute_deviation(delta_ms)
    # "For normally distributed data k is taken to be 1.4826" - https://en.wikipedia.org/wiki/Median_absolute_deviation
    robust_sigma = 1.4826 * mad

    if robust_sigma < 1e-8:
        # If we are here then that means the data is basically exactly homogenous, and therefore we need to hardcode the
        # min/max outlier ranges. Here we do this as plus/minus 1ms. Honestly if this ever happens there will likely be
        # no outliers, but it is more principled than just setting it as the min/max delta value which might be the
        # exact same value for syntheticlly timestamped data.
        focus_min = delta_ms.min() - 1
        focus_max = delta_ms.max() + 1
    else:
        # TODO(Jack): Should the outlier sigma be configurable? Hard coded works for now...
        median_delta = delta_ms.median()
        outlier_sigma = 6.0

        focus_min = median_delta - outlier_sigma * robust_sigma
        focus_max = median_delta + outlier_sigma * robust_sigma

    low_outlier_mask = delta_ms < focus_min
    high_outlier_mask = delta_ms > focus_max

    inlier_mask = ~(low_outlier_mask | high_outlier_mask)
    outlier_mask = low_outlier_mask | high_outlier_mask

    low_outlier_count = int(low_outlier_mask.sum())
    high_outlier_count = int(high_outlier_mask.sum())

    delta_fig = go.Figure()
    delta_fig.add_trace(
        go.Scattergl(
            x=rows.loc[inlier_mask, "time_s"],
            y=rows.loc[inlier_mask, "delta_ms"],
            mode="markers",
            marker=dict(size=4, opacity=0.7),
            name="Inliers",
        )
    )

    if low_outlier_count > 0:
        low_outliers = rows.loc[low_outlier_mask].copy()

        delta_fig.add_trace(
            go.Scattergl(
                x=low_outliers["time_s"],
                y=[0.02] * len(low_outliers),
                yaxis="y2",
                mode="markers",
                marker=dict(size=8, symbol="triangle-down"),
                name="Low outliers",
            )
        )

    if high_outlier_count > 0:
        high_outliers = rows.loc[high_outlier_mask].copy()

        delta_fig.add_trace(
            go.Scattergl(
                x=high_outliers["time_s"],
                y=[0.98] * len(high_outliers),
                yaxis="y2",
                mode="markers",
                marker=dict(size=8, symbol="triangle-up"),
                name="High outliers",
            )
        )

    outlier_count = low_outlier_count + high_outlier_count
    inlier_values = rows.loc[inlier_mask, "delta_ms"]
    outlier_values = rows.loc[outlier_mask, "delta_ms"]
    inlier_stats = format_stats("Inliers", inlier_values)
    outlier_stats = format_stats("Outliers", outlier_values)

    subtitle = (
        f"{outlier_count} outlier intervals shown at plot edges "
        f"({low_outlier_count} below, {high_outlier_count} above)<br>"
        f"{inlier_stats}<br>"
        f"{outlier_stats}"
    )

    delta_fig.update_layout(
        title=f"{label} measurement interval time<br><sup>{subtitle}</sup>",
        xaxis=dict(title="Time [s]"),
        yaxis=dict(title="Interval [ms]"),
        # NOTE(Jack): Using this second axis with a "paper" like coordinate system lets us hardcode the location of the
        # inlier/outlier points which saves us from having to calculate it each time. Not like that would be complex, it
        # is simply that this solution has fewer moving parts and code to maintain.
        yaxis2=dict(
            overlaying="y", range=[0, 1], visible=False, showgrid=False, zeroline=False
        ),
        margin=dict(t=150),
        showlegend=outlier_count > 0,
    )

    histogram_fig = go.Figure()
    histogram_fig.add_trace(go.Histogram(x=inlier_values, nbinsx=100, name="Inliers"))

    if low_outlier_count > 0:
        low_outliers = rows.loc[low_outlier_mask].copy()

        histogram_fig.add_trace(
            go.Scatter(
                x=low_outliers["delta_ms"],
                y=[0.98] * len(low_outliers),
                xaxis="x2",
                yaxis="y2",
                mode="markers",
                marker=dict(size=8, symbol="triangle-down"),
                name="Low outliers",
            )
        )

    if high_outlier_count > 0:
        high_outliers = rows.loc[high_outlier_mask].copy()

        histogram_fig.add_trace(
            go.Scatter(
                x=high_outliers["delta_ms"],
                y=[0.98] * len(high_outliers),
                xaxis="x2",
                yaxis="y2",
                mode="markers",
                marker=dict(size=8, symbol="triangle-up"),
                name="High outliers",
            )
        )

    histogram_fig.update_layout(
        title=f"{label} measurement interval histogram",
        xaxis=dict(title="Inlier interval [ms]"),
        xaxis2=dict(
            title="Outlier interval [ms]",
            overlaying="x",
            side="top",
            showgrid=False,
            zeroline=False,
        ),
        yaxis=dict(title="Count"),
        yaxis2=dict(
            overlaying="y", range=[0, 1], visible=False, showgrid=False, zeroline=False
        ),
        showlegend=outlier_count > 0,
    )

    return delta_fig, histogram_fig
