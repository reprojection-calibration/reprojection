import plotly.graph_objects as go

import numpy as np

canonical_camera_axes = np.array(
    [
        [0, -1, 0],
        [0, 0, -1],
        [1, 0, 0],
    ]
)


def transform_to_pose_axes(tf):
    position = np.array([tf[:3, 3]])

    x_col = np.array([tf[:3, 0]])
    x_line = np.concatenate((position, position + x_col), axis=0).T
    y_col = np.array([tf[:3, 1]])
    y_line = np.concatenate((position, position + y_col), axis=0).T
    z_col = np.array([tf[:3, 2]])
    z_line = np.concatenate((position, position + z_col), axis=0).T

    return (x_line, "red"), (y_line, "green"), (z_line, "blue")


def plot_pose_axes(tf, fig=None):
    pose_axes = transform_to_pose_axes(tf)

    if fig is None:
        fig = go.Figure()

    for axes, color in pose_axes:
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


def gravity_aligned_pose(origin, position):
    delta = origin - position

    # TODO(Jack): Handle case where direction has not magnitude

    forward_direction = delta / np.linalg.norm(delta)
    gravity_direction = np.array([0, 0, -1])

    xxx = np.cross(forward_direction, gravity_direction)

    # TODO(Jack): Handle the case where the cross product here is zero!

    xxx_direction = xxx / np.linalg.norm(xxx)
    yyy = np.cross(forward_direction, xxx_direction)
    yyy_direction = yyy / np.linalg.norm(yyy)

    tf = np.identity(4)
    tf[:3, 0] = forward_direction
    tf[:3, 1] = xxx_direction
    tf[:3, 2] = yyy_direction
    tf[:3, :3] =  np.matmul(tf[:3, :3], np.linalg.inv(canonical_camera_axes))
    tf[:3, 3] = position

    return tf


origin = np.array([0, 0, 0])
position = np.array([1, 1, 1])
tf = gravity_aligned_pose(origin, position)
fig = plot_pose_axes(tf)

position = np.array([-1, 1, 1])
tf = gravity_aligned_pose(origin, position)
fig = plot_pose_axes(tf, fig)

position = np.array([1, -1, 1])
tf = gravity_aligned_pose(origin, position)
fig = plot_pose_axes(tf, fig)

position = np.array([1, 1, -1])
tf = gravity_aligned_pose(origin, position)
fig = plot_pose_axes(tf, fig)

# Plot a point at the origin for better viewing context.
fig.add_trace(
    go.Scatter3d(
        x=[0],
        y=[0],
        z=[0],
        mode="markers",
    )
)

fig.update_layout(
    scene=dict(
        aspectmode="cube",
        xaxis_title="X",
        yaxis_title="Y",
        zaxis_title="Z",
        xaxis=dict(range=[-3, 3]),
        yaxis=dict(range=[-3, 3]),
        zaxis=dict(range=[-3, 3]),
    )
)
fig.show()
