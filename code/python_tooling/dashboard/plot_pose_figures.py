import plotly.graph_objects as go

# TODO(Jack): There is a lot of copy and paste in this file! We can eliminate this, only question is is it worth it?

def plot_rotation_figure(data, fig=None, legendgroup=None, marker='circle'):
    times = [n['timestamp_ns'] for n in data]
    rotations_x = [d['rx'] for d in data]
    rotations_y = [d['ry'] for d in data]
    rotations_z = [d['rz'] for d in data]

    if fig is None:
        fig = go.Figure()

    fig.add_scatter(x=times, y=rotations_x, marker=dict(symbol=marker, color='rgb(255, 0, 0)'), mode='lines+markers',
                    name='rx',
                    legendgroup=legendgroup)
    fig.add_scatter(x=times, y=rotations_y, marker=dict(symbol=marker, color='rgb(18, 174, 0)'), mode='lines+markers',
                    name='ry',
                    legendgroup=legendgroup)
    fig.add_scatter(x=times, y=rotations_z, marker=dict(symbol=marker, color='rgb(0, 0, 255)'), mode='lines+markers',
                    name='rz',
                    legendgroup=legendgroup)
    fig.update_layout(
        title='Axis Angle Rotation',
        xaxis_title='Time (ns)',
        yaxis_title='(rad)',
    )

    return fig


def plot_translation_figure(data, fig=None, legendgroup=None, marker='circle'):
    times = [n['timestamp_ns'] for n in data]
    translation_x = [d['x'] for d in data]
    translation_y = [d['y'] for d in data]
    translation_z = [d['z'] for d in data]

    if fig is None:
        fig = go.Figure()

    fig.add_scatter(x=times, y=translation_x, marker=dict(symbol=marker, color='rgb(255, 0, 0)'),
                    mode='lines+markers',
                    name='x',
                    legendgroup=legendgroup)
    fig.add_scatter(x=times, y=translation_y, marker=dict(symbol=marker, color='rgb(18, 174, 0)'),
                    mode='lines+markers', name='y',
                    legendgroup=legendgroup)
    fig.add_scatter(x=times, y=translation_z, marker=dict(symbol=marker, color='rgb(0, 0, 255)'),
                    mode='lines+markers', name='z',
                    legendgroup=legendgroup)
    fig.update_layout(
        title='Translation',
        xaxis_title='Time (ns)',
        yaxis_title='(m)',
    )

    return fig
