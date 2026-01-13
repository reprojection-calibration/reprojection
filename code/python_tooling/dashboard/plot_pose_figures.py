import plotly.graph_objects as go


# TODO(Jack): Does this function belong here and should we test it?
def extract_timestamps_and_poses_sorted(frames, pose_type):
    timestamps = []
    poses = []
    for timestamp in sorted(frames):
        frame_i = frames[timestamp]

        if 'poses' not in frame_i:
            continue

        if pose_type not in frame_i['poses']:
            continue

        timestamps.append(timestamp)
        poses.append(frame_i['poses'][pose_type])

    return timestamps, poses


# Starts at zero and denominated in seconds. Input must be sorted!
def to_human_readable(timestamps_ns):
    t0 = timestamps_ns[0]
    timestamps_human_readable = [(t - t0) / 1e9 for t in timestamps_ns]

    return timestamps_human_readable


# TODO(Jack): This function possible belongs in another file because it is not really related to the poses only.
def calculate_ticks_from_timestamps(timestamps_ns, step=5):
    timestamps_s = to_human_readable(timestamps_ns)

    max_time = timestamps_s[-1]
    target_times = range(0, int(max_time) + 1, step)

    # NOTE(Jack): The idxs are consumed by the slider bar which is indexed in ticker/counter space  and the seconds
    # output is consumed by the pose graphs where we plot data indexed by time.
    tickvals_idx = []
    tickvals_s = []
    ticktext = []
    for target in target_times:
        closest_index = min(
            range(len(timestamps_s)),
            key=lambda i: abs(timestamps_s[i] - target)
        )

        tickvals_idx.append(closest_index)
        tickvals_s.append(timestamps_s[closest_index])
        ticktext.append(f"{target}s")

    return tickvals_idx, tickvals_s, ticktext


def plot_pose_figure(timestamps_ns, data, title, yaxis_title, fig=None, legendgroup=None, marker='circle', x_name='x',
                     y_name='y', z_name='z'):
    if len(timestamps_ns) != len(data) or len(timestamps_ns) == 0:
        return {}

    # TODO(Jack): Should we raise an exception here because this is a real error.
    # Expect either [rz, ry, rz] or [x, y, z] - at this time nothing else is valid!
    if len(data[0]) != 3:
        return {}

    x = [d[0] for d in data]
    y = [d[1] for d in data]
    z = [d[2] for d in data]

    if fig is None:
        fig = go.Figure()

    timestamps_ns = [int(t) for t in timestamps_ns]
    timestamps_s = to_human_readable(timestamps_ns)

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
        yaxis_title=yaxis_title,
        xaxis=dict(
            title="Time (s)",
            tickmode="array",
            tickvals=tickvals_s,
            ticktext=ticktext,
        ),
    )

    return fig
