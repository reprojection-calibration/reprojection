import plotly.graph_objects as go


def plot_rotation_figure(data, fig=None, legendgroup=None, marker='circle'):
    times = [n['timestamp_ns'] for n in data]

    # TODO(Jack): Eliminate copy and paste here!
    # TODO(Jack): Use numpy arrays here if possible?
    rotations_x = [d['rx'] for d in data]
    rotations_y = [d['ry'] for d in data]
    rotations_z = [d['rz'] for d in data]

    if fig is None:
        fig = go.Figure()
    fig.add_scatter(x=times, y=rotations_x, marker=dict(symbol=marker, color='rgb(255, 0, 0)'), mode='lines+markers',
                    name='rx',
                    legendgroup=legendgroup)
    fig.add_scatter(x=times, y=rotations_y, marker=dict(symbol=marker, color='rgb(18, 174, 0)'), mode='lines+markers', name='ry',
                    legendgroup=legendgroup)
    fig.add_scatter(x=times, y=rotations_z, marker=dict(symbol=marker, color='rgb(0, 0, 255)'), mode='lines+markers', name='rz',
                    legendgroup=legendgroup)
    fig.update_layout(
        xaxis_title='Time (ns)',
        yaxis_title='Axis Angle Rotation',
    )

    return fig


def plot_translation_figure(data, fig=None, legendgroup=None, marker='circle'):
    times = [n['timestamp_ns'] for n in data]

    translation_x = [d['x'] for d in data]
    translation_y = [d['y'] for d in data]
    translation_z = [d['z'] for d in data]

    # TODO(Jack): Add legend group
    if fig is None:
        fig = go.Figure()
    fig.add_scatter(x=times, y=translation_x, marker=dict(symbol=marker, color='rgb(255, 0, 0)'),
                    mode='lines+markers',
                    name='x',
                    legendgroup=legendgroup)
    fig.add_scatter(x=times, y=translation_y, marker=dict(symbol=marker, color='rgb(18, 174, 0)' ),
                    mode='lines+markers', name='y',
                    legendgroup=legendgroup)
    fig.add_scatter(x=times, y=translation_z, marker=dict(symbol=marker, color='rgb(0, 0, 255)' ),
                    mode='lines+markers', name='z',
                    legendgroup=legendgroup)
    fig.update_layout(
        xaxis_title='Time (ns)',
        yaxis_title='Translation (m)',
    )

    return fig
