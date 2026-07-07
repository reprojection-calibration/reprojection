import plotly.graph_objects as go


def _mean_absolute_deviation(values):
    if values.empty:
        return None

    mean = values.mean()

    return (values - mean).abs().mean()


def _format_stats(name, values):
    if values.empty:
        return f"{name}: N/A"

    mean = values.mean()
    mad = _mean_absolute_deviation(values)

    return f"{name}: mean={mean:.3f} ms, mean abs dev={mad:.3f} ms"


def measurement_delta_time_figures(df, label="Measurement", outlier_sigma=6.0):
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

    # The first measurement has no previous sample.
    rows = rows.dropna(subset=["delta_ms"])

    assert not rows.empty, "Need at least two timestamps to compute deltas"

    delta_ms = rows["delta_ms"]

    #
    # Robust outlier detection.
    #
    median_delta = delta_ms.median()
    mad_from_median = (delta_ms - median_delta).abs().median()
    robust_sigma = 1.4826 * mad_from_median

    if robust_sigma == 0:
        focus_min = delta_ms.min()
        focus_max = delta_ms.max()

        low_outlier_mask = rows["delta_ms"] < focus_min
        high_outlier_mask = rows["delta_ms"] > focus_max
    else:
        focus_min = median_delta - outlier_sigma * robust_sigma
        focus_max = median_delta + outlier_sigma * robust_sigma

        low_outlier_mask = rows["delta_ms"] < focus_min
        high_outlier_mask = rows["delta_ms"] > focus_max

    inlier_mask = ~(low_outlier_mask | high_outlier_mask)
    outlier_mask = low_outlier_mask | high_outlier_mask

    #
    # Summary statistics for timeseries title only.
    #
    inlier_values = rows.loc[inlier_mask, "delta_ms"]
    outlier_values = rows.loc[outlier_mask, "delta_ms"]

    inlier_stats = _format_stats("Inliers", inlier_values)
    outlier_stats = _format_stats("Outliers", outlier_values)

    #
    # Timeseries axis range based on inliers.
    #
    if inlier_values.empty:
        display_min = delta_ms.min()
        display_max = delta_ms.max()
    else:
        display_min = inlier_values.min()
        display_max = inlier_values.max()

    padding = 0.1 * (display_max - display_min)
    if padding == 0:
        padding = max(0.01, abs(display_min) * 0.001)

    delta_range = [
        display_min - padding,
        display_max + padding,
        ]

    delta_span = delta_range[1] - delta_range[0]
    edge_offset = 0.02 * delta_span

    lower_edge_value = delta_range[0] + edge_offset
    upper_edge_value = delta_range[1] - edge_offset

    low_outlier_count = int(low_outlier_mask.sum())
    high_outlier_count = int(high_outlier_mask.sum())
    outlier_count = low_outlier_count + high_outlier_count

    subtitle = (
        f"{outlier_count} outlier intervals shown at plot edges "
        f"({low_outlier_count} below, {high_outlier_count} above)<br>"
        f"{inlier_stats}<br>"
        f"{outlier_stats}"
    )

    #
    # Delta time series
    #
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
                y=[lower_edge_value] * len(low_outliers),
                mode="markers",
                marker=dict(
                    size=8,
                    opacity=0.95,
                    symbol="triangle-down",
                ),
                name="Low outliers",
                customdata=low_outliers["delta_ms"],
                hovertemplate=(
                    "Measurement time [s]: %{x}<br>"
                    "Actual delta time [ms]: %{customdata:.6f}<br>"
                    "Shown at lower edge"
                    "<extra></extra>"
                ),
            )
        )

    if high_outlier_count > 0:
        high_outliers = rows.loc[high_outlier_mask].copy()

        delta_fig.add_trace(
            go.Scattergl(
                x=high_outliers["time_s"],
                y=[upper_edge_value] * len(high_outliers),
                mode="markers",
                marker=dict(
                    size=8,
                    opacity=0.95,
                    symbol="triangle-up",
                ),
                name="High outliers",
                customdata=high_outliers["delta_ms"],
                hovertemplate=(
                    "Measurement time [s]: %{x}<br>"
                    "Actual delta time [ms]: %{customdata:.6f}<br>"
                    "Shown at upper edge"
                    "<extra></extra>"
                ),
            )
        )

    delta_fig.update_layout(
        title=f"{label} measurement delta time<br><sup>{subtitle}</sup>",
        xaxis=dict(title="Measurement time [s]"),
        yaxis=dict(
            title="Delta time [ms]",
            range=delta_range,
        ),
        margin=dict(t=150),
        showlegend=outlier_count > 0,
    )

    #
    # Histogram
    #
    # Important: plot all data here, including outliers.
    #
    histogram_fig = go.Figure()

    histogram_fig.add_trace(
        go.Histogram(
            x=rows["delta_ms"],
            nbinsx=100,
            name="All intervals",
        )
    )

    histogram_fig.update_layout(
        title=f"{label} measurement interval histogram",
        xaxis=dict(title="Delta time [ms]"),
        yaxis=dict(title="Frequency"),
        margin=dict(t=80),
        showlegend=False,
    )

    return delta_fig, histogram_fig