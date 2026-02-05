import plotly.graph_objects as go

import numpy as np


def transform_to_pose_axes(tf):
    position = np.array([tf[:3, 3]])

    x_col = np.array([tf[:3, 0]])
    x_line = np.concatenate((position, position + x_col), axis=0).T
    y_col = np.array([tf[:3, 1]])
    y_line = np.concatenate((position, position + y_col), axis=0).T
    z_col = np.array([tf[:3, 2]])
    z_line = np.concatenate((position, position + z_col), axis=0).T

    return (x_line, "red"), (y_line, "green"), (z_line, "blue")


def plot_pose_axes(pose_axes, fig=None):
    if fig is None:
        fig = go.Figure()

    for axes, color in pose_axes:
        print(color)
        fig.add_trace(
            go.Scatter3d(
                x=axes[0, :],
                y=axes[1, :],
                z=axes[2, :],
                mode="lines",
                line=dict(color=color, width=5),
            )
        )

    return fig


tf = np.identity(4)
tf[:3, 3] = [1, 1, 1]
pose_axes = transform_to_pose_axes(tf)

fig = plot_pose_axes(pose_axes)

fig.update_layout(
    scene=dict(
        xaxis_title="X",
        yaxis_title="Y",
        zaxis_title="Z",
        xaxis=dict(range=[-3, 3]),
        yaxis=dict(range=[-3, 3]),
        zaxis=dict(range=[-3, 3]),
    )
)
fig.show()
