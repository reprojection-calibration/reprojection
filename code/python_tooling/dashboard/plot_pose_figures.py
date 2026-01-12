import plotly.graph_objects as go


# TODO(Jack): Does this function belong here and should we test it?
def extract_timestamps_and_poses(frames, pose_type):
    timestamps = []
    poses = []
    for timestamp, frame_data in frames.items():
        if 'poses' not in frame_data:
            continue

        if pose_type not in frame_data['poses']:
            continue

        timestamps.append(timestamp)
        poses.append(frame_data['poses'][pose_type])

    return timestamps, poses


def plot_pose_figure(timestamps, data, title, yaxis_title, fig=None, legendgroup=None, marker='circle', x_name='x',
                     y_name='y', z_name='z'):
    if len(timestamps) != len(data) or len(timestamps) == 0:
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

    fig.add_scatter(x=timestamps, y=x, marker=dict(symbol=marker, color='rgb(255, 0, 0)'), mode='lines+markers',
                    name=x_name,
                    legendgroup=legendgroup)
    fig.add_scatter(x=timestamps, y=y, marker=dict(symbol=marker, color='rgb(18, 174, 0)'), mode='lines+markers',
                    name=y_name,
                    legendgroup=legendgroup)
    fig.add_scatter(x=timestamps, y=z, marker=dict(symbol=marker, color='rgb(0, 0, 255)'), mode='lines+markers',
                    name=z_name,
                    legendgroup=legendgroup)
    fig.update_layout(
        title=title,
        xaxis_title='Time (ns)',
        yaxis_title=yaxis_title,
    )

    return fig
