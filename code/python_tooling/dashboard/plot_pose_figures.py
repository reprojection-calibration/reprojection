import plotly.graph_objects as go

def plot_rotation_figure(data, fig = None):
    times = [n['timestamp_ns'] for n in data]

    # TODO(Jack): Eliminate copy and paste here!
    # TODO(Jack): Use numpy arrays here if possible?
    rotations_x = [d['rx'] for d in data]
    rotations_y = [d['ry'] for d in data]
    rotations_z = [d['rz'] for d in data]

    if fig is None:
        fig = go.Figure()
    fig.add_scatter(x=times, y=rotations_x, mode='lines+markers', name='X', legendgroup='ExternalPose')
    fig.add_scatter(x=times, y=rotations_y, mode='lines+markers', name='Y', legendgroup='ExternalPose')
    fig.add_scatter(x=times, y=rotations_z, mode='lines+markers', name='Z', legendgroup='ExternalPose')
    fig.update_layout(
        xaxis_title='Time(ns)',
        yaxis_title='Axis Angle Rotation',
        legend_title_text='Sources'
    )

    return fig

def plot_translation_figure(data, fig = None):
    times = [n['timestamp_ns'] for n in data]

    translation_x = [d['x'] for d in data]
    translation_y = [d['y'] for d in data]
    translation_z = [d['z'] for d in data]

    # TODO(Jack): Add legend group
    if fig is None:
        fig = go.Figure()
    fig.add_scatter(x=times, y=translation_x, mode='lines+markers', name='rx')
    fig.add_scatter(x=times, y=translation_y, mode='lines+markers', name='ry')
    fig.add_scatter(x=times, y=translation_z, mode='lines+markers', name='rz')
    fig.update_layout(
        xaxis_title='Time(ns)',
        yaxis_title='Translation(m)'
    )

    return fig