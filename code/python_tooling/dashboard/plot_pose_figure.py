import plotly.graph_objects as go

from time_handling import timestamps_to_elapsed_seconds, calculate_ticks_from_timestamps


# NOTE(Jack): Think about it this way. The moment that we have two separate arrays we cannot/should not ever sort them.
# They should already be sorted at the time when their correspondence was still programmatically enforced. To do the
# sorting after they have been separated from each other would be crazy. That means this function requires the input
# timestamps and data to already be sorted!
def plot_pose_figure(timestamps_ns, data, title, yaxis_title, fig=None, legendgroup=None, marker='circle', x_name='x',
                     y_name='y', z_name='z', ymin=-3.15, ymax=3.15):
    if len(timestamps_ns) != len(data) or len(timestamps_ns) == 0:
        return {}

    # TODO(Jack): Should we raise an exception here because this is a real error?
    # Expect either [rz, ry, rz] or [x, y, z] - at this time nothing else is valid!
    if len(data[0]) != 3:
        return {}

    x = [d[0] for d in data]
    y = [d[1] for d in data]
    z = [d[2] for d in data]

    if fig is None:
        fig = go.Figure()

    # TODO(Jack): When we get the data from the store the timestamps are strings, so we need to convert them to int
    #  here. Should we deal with this programmatically and convert them to ints when they get loaded into the store?
    timestamps_ns = [int(t) for t in timestamps_ns]
    timestamps_s = timestamps_to_elapsed_seconds(timestamps_ns)

    fig.add_scatter(x=timestamps_s, y=x, marker=dict(symbol=marker, color='rgb(255, 0, 0)'), mode='markers',
                    name=x_name,
                    legendgroup=legendgroup)
    fig.add_scatter(x=timestamps_s, y=y, marker=dict(symbol=marker, color='rgb(18, 174, 0)'), mode='markers',
                    name=y_name,
                    legendgroup=legendgroup)
    fig.add_scatter(x=timestamps_s, y=z, marker=dict(symbol=marker, color='rgb(0, 0, 255)'), mode='markers',
                    name=z_name,
                    legendgroup=legendgroup)

    _, tickvals_s, ticktext = calculate_ticks_from_timestamps(timestamps_ns)
    fig.update_layout(
        title=title,
        xaxis=dict(
            title="Time (s)",
            tickmode="array",
            tickvals=tickvals_s,
            ticktext=ticktext,
        ),
        yaxis=dict(
            title=yaxis_title,
            range=[ymin, ymax],
        ),
    )

    return fig
