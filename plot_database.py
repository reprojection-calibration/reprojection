import sqlite3
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict, deque
from scipy.spatial.transform import Rotation as R

DB_PATH = "/home/stable-genius-gram/data/cvg.cit.tum.de_visual-inertial-dataset/dataset-calib-imu4_512_16.db3"
MAX_POSES = 900
AXIS_LENGTH = 0.25  # Length of camera axes in world units

# -------------------------------------------------------
# Load poses from SQLite
# -------------------------------------------------------
def load_poses(db_path):
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute("""
                SELECT timestamp_ns, sensor_name, rx, ry, rz, x, y, z
                FROM initial_camera_poses
                ORDER BY timestamp_ns ASC
                """)
    rows = cur.fetchall()
    conn.close()
    return rows

# -------------------------------------------------------
# Fast incremental plotting with lines and orientation axes
# -------------------------------------------------------
def plot_poses(rows):
    pose_buffers = defaultdict(lambda: deque(maxlen=MAX_POSES))
    sensors = list(sorted(set(r[1] for r in rows)))

    # Precompute static plot bounds
    positions = np.array([[r[5], r[6], r[7]] for r in rows])
    min_xyz = positions.min(axis=0)
    max_xyz = positions.max(axis=0)
    center = 0.5 * (min_xyz + max_xyz)
    radius = 0.5 * np.max(max_xyz - min_xyz) * 1.2

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    plt.ion()

    # Pre-create trajectory lines, orientation axes, and text annotations
    traj_lines = {}
    axes_lines = {}
    text_annotations = {}
    colors = plt.cm.tab10.colors  # Up to 10 distinct colors
    for i, sensor in enumerate(sensors):
        traj_lines[sensor], = ax.plot([], [], [], '-', color=colors[i % 10], label=sensor)
        axes_lines[sensor] = [
            ax.plot([], [], [], 'r')[0],  # X axis
            ax.plot([], [], [], 'g')[0],  # Y axis
            ax.plot([], [], [], 'b')[0],  # Z axis
        ]
        text_annotations[sensor] = ax.text(0, 0, 0, '', color='k', fontsize=8)

    # Static axes limits
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)
    ax.set_box_aspect([1, 1, 1])
    ax.set_autoscale_on(False)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Initial Camera Poses (IDs Only)")
    ax.legend(loc="upper right")

    # Iterate through poses
    for idx, row in enumerate(rows):
        timestamp, sensor, rx, ry, rz, x, y, z = row
        pose_buffers[sensor].append((np.array([x, y, z]), np.array([rx, ry, rz]), idx))

        # Update trajectory line
        traj = np.array([p[0] for p in pose_buffers[sensor]])
        traj_lines[sensor].set_data(traj[:,0], traj[:,1])
        traj_lines[sensor].set_3d_properties(traj[:,2])

        # Update orientation axes for latest pose
        pos, rvec, pid = pose_buffers[sensor][-1]
        rot = R.from_rotvec(rvec)
        R_mat = rot.as_matrix()
        axes = R_mat * AXIS_LENGTH

        # X axis (red)
        axes_lines[sensor][0].set_data([pos[0], pos[0]+axes[0,0]],
                                       [pos[1], pos[1]+axes[1,0]])
        axes_lines[sensor][0].set_3d_properties([pos[2], pos[2]+axes[2,0]])
        # Y axis (green)
        axes_lines[sensor][1].set_data([pos[0], pos[0]+axes[0,1]],
                                       [pos[1], pos[1]+axes[1,1]])
        axes_lines[sensor][1].set_3d_properties([pos[2], pos[2]+axes[2,1]])
        # Z axis (blue)
        axes_lines[sensor][2].set_data([pos[0], pos[0]+axes[0,2]],
                                       [pos[1], pos[1]+axes[1,2]])
        axes_lines[sensor][2].set_3d_properties([pos[2], pos[2]+axes[2,2]])

        # Update ID annotation
        text_annotations[sensor].set_position((pos[0], pos[1]))
        text_annotations[sensor].set_3d_properties(pos[2])
        text_annotations[sensor].set_text(f"ID:{pid}")

        plt.draw()
        plt.pause(0.01)

    plt.ioff()
    plt.show()


# -------------------------------------------------------
# Entry point
# -------------------------------------------------------
if __name__ == "__main__":
    rows = load_poses(DB_PATH)
    if not rows:
        print("No poses found.")
    else:
        plot_poses(rows)