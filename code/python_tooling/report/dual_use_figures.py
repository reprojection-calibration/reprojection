import plotly.graph_objects as go


def mean_absolute_deviation(values):
    if values.empty:
        return None

    mean = values.mean()

    return (values - mean).abs().mean()


def format_stats(name, values):
    if values.empty:
        return f"{name}: N/A"

    mean = values.mean()
    mad = mean_absolute_deviation(values)

    return f"{name}: mean={mean:.3f} ms, mean abs dev={mad:.3f} ms"


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
    median_delta = delta_ms.median()
    mean_abs_dev = (delta_ms - median_delta).abs().median()
    robust_sigma = 1.4826 * mean_abs_dev

    if robust_sigma < 1e-8:
        # If we are here then that means the data is basically exactly homogenous, and therefore we need to hardcode the
        # min/max outlier ranges. Here we do this as plus/minus 1ms. Honestly if this ever happens there will likely be
        # no outliers, but it is more principled than just setting it as the min/max delta value which might be the
        # exact same value for syntheticlly timestamped data.
        focus_min = delta_ms.min() - 1
        focus_max = delta_ms.max() + 1
    else:
        # TODO(Jack): Does this need to be configurable? For now hardcoding works :)
        outlier_sigma = 6.0
        focus_min = median_delta - outlier_sigma * robust_sigma
        focus_max = median_delta + outlier_sigma * robust_sigma

    low_outlier_mask = delta_ms < focus_min
    high_outlier_mask = delta_ms > focus_max

    inlier_mask = ~(low_outlier_mask | high_outlier_mask)
    outlier_mask = low_outlier_mask | high_outlier_mask

    inlier_values = rows.loc[inlier_mask, "delta_ms"]
    outlier_values = rows.loc[outlier_mask, "delta_ms"]

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
                marker=dict(
                    size=8,
                    opacity=0.95,
                    symbol="triangle-down",
                ),
                name="Low outliers",
                customdata=low_outliers["delta_ms"],
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
                marker=dict(
                    size=8,
                    opacity=0.95,
                    symbol="triangle-up",
                ),
                name="High outliers",
                customdata=high_outliers["delta_ms"],
            )
        )

    outlier_count = low_outlier_count + high_outlier_count
    inlier_stats = format_stats("Inliers", inlier_values)
    outlier_stats = format_stats("Outliers", outlier_values)
    subtitle = (
        f"{outlier_count} outlier intervals shown at plot edges "
        f"({low_outlier_count} below, {high_outlier_count} above)<br>"
        f"{inlier_stats}<br>"
        f"{outlier_stats}"
    )

    # The timeseries plot only plots the inliers so we need to calculate the y-axis limits here. The histogram plots all
    # the data with no differentiation between inlier or outlier.
    delta_range = [
        delta_ms.min() if inlier_values.empty else inlier_values.min(),
        delta_ms.max() if inlier_values.empty else inlier_values.max(),
    ]

    delta_fig.update_layout(
        title=f"{label} measurement delta time<br><sup>{subtitle}</sup>",
        xaxis=dict(title="Measurement time [s]"),
        yaxis=dict(
            title="Delta time [ms]",
            range=delta_range,
        ),
        # Using this second axis with a "paper" like coordinate system lets us hardcode the location of the
        # inlier/outlier points which saves us from having to calculate it each time. Not like that would be complex, it
        # is simply that this solution has fewer moving parts and code to maintain.
        yaxis2=dict(
            overlaying="y",
            range=[0, 1],
            visible=False,
            showgrid=False,
            zeroline=False,
        ),
        margin=dict(t=150),
        showlegend=outlier_count > 0,
    )

    histogram_fig = go.Figure()
    histogram_fig.add_trace(
        go.Histogram(
            x=delta_ms,
            nbinsx=100,
            name="All intervals",
        )
    )

    histogram_fig.update_layout(
        title=f"{label} measurement interval histogram",
        xaxis=dict(title="Delta time [ms]"),
        yaxis=dict(title="Frequency"),
        showlegend=False,
    )

    return delta_fig, histogram_fig
