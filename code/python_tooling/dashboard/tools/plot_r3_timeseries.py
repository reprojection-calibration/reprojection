from dataclasses import dataclass

import plotly.graph_objects as go

from dashboard.tools.time_handling import (
    calculate_ticks_from_timestamps,
    timestamps_to_elapsed_seconds,
)


# TODO(Jack): We could pass part of this directly to the plot initializer so that we have more than just the x-axis
#  initialized.
@dataclass
class R3TimeseriesFigureConfig:
    title: str = ""
    yaxis_title: str = ""
    x_name: str = "x"
    y_name: str = "y"
    z_name: str = "z"
    ymin: float = "0"
    ymax: float = "1"


# NOTE(Jack): Think about it this way. The moment that we have two separate arrays we cannot/should not ever sort them.
# They should already be sorted at the time when their correspondence was still programmatically enforced. To do the
# sorting after they have been separated from each other would be crazy. That means this function requires the input
# timestamps and data to already be sorted!
def build_r3_timeseries_figure(
    timestamps_ns,
    data,
    config: R3TimeseriesFigureConfig,
    fig=None,
):
    if len(timestamps_ns) != len(data) or len(timestamps_ns) == 0:
        return {}

    # WARN(Jack): We only check the dimension of the first element which really makes this a half assed check...
    # Expect either [rz, ry, rz] or [x, y, z] - at this time nothing else is valid!
    if len(data[0]) != 3:
        return {}

    # TODO USE NUMPY!
    x = [d[0] for d in data]
    y = [d[1] for d in data]
    z = [d[2] for d in data]

    if fig is None:
        fig = go.Figure()

    # TODO(Jack): When we get the data from the store the timestamps are strings, so we need to convert them to int
    #  here. Should we deal with this programmatically and convert them to ints when they get loaded into the store?
    timestamps_ns = [int(t) for t in timestamps_ns]
    timestamps_s = timestamps_to_elapsed_seconds(timestamps_ns)

    # NOTE(Jack): We use go.Scattergl() because it is way way faster than a regular scatter plot with lots of points.
    fig.add_trace(
        go.Scattergl(
            x=timestamps_s,
            y=x,
            marker=dict(color="rgb(255, 0, 0)"),
            mode="markers",
            name=config.x_name,
        )
    )
    fig.add_trace(
        go.Scattergl(
            x=timestamps_s,
            y=y,
            marker=dict(color="rgb(18, 174, 0)"),
            mode="markers",
            name=config.y_name,
        )
    )
    fig.add_trace(
        go.Scattergl(
            x=timestamps_s,
            y=z,
            marker=dict(color="rgb(0, 0, 255)"),
            mode="markers",
            name=config.z_name,
        )
    )

    fig.update_layout(
        title=config.title,
        yaxis=dict(
            title=config.yaxis_title,
            range=[config.ymin, config.ymax],
        ),
    )

    return fig


# WARN(Jack): Timestamps must be sorted! Can we programmatically assert this?
# TODO(Jack): Naming! Timeseries plot is too generic! We are building a properly sized x-axis for all time series camera
#  frame data.
def timeseries_plot(timestamps_ns, step=5):
    _, tickvals_s, ticktext = calculate_ticks_from_timestamps(timestamps_ns, step)

    fig = go.Figure()
    if len(tickvals_s) == 0 or len(ticktext) == 0:
        return fig

    # WARN(Jack): The way our calculate_ticks_from_timestamps() from timestamps method works (and needs to work I think)
    # means that it will have one less tick than it really needs to cover the entire data (this is because it wants to
    # also match frame idxs not just times). Therefore, when setting the range below we arbitrarily add one step to the
    # max value. If there was a more programmatic way to do this (i.e. inside calculate_ticks_from_timestamps()) then
    # we should consider doing that!
    fig.update_layout(
        xaxis=dict(
            title="Time (s)",
            range=[tickvals_s[0], tickvals_s[-1] + step],
            tickmode="array",
            tickvals=tickvals_s,
            ticktext=ticktext,
        ),
    )

    return fig
