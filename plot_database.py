import sqlite3
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict, deque
from scipy.spatial.transform import Rotation as R

DB_PATH = "/home/stable-genius-gram/data/cvg.cit.tum.de_visual-inertial-dataset/dataset-calib-imu4_512_16.db3"
MAX_POSES = 50
AXIS_LENGTH = 0.5  # Length of camera axes in world units

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
# Plot poses incrementally with orientations
# -------------------------------------------------------
def plot_poses(rows):
    pose_buffers = defaultdict(lambda: deque(maxlen=MAX_POSES))

    # Precompute static plot bounds
    positions = np.array([[r[5], r[6], r[7]] for r in rows])
    min_xyz = positions.min(axis=0)
    max_xyz = positions.max(axis=0)
    center = 0.5 * (min_xyz + max_xyz)
    radius = 0.5 * np.max(max_xyz - min_xyz) * 1.2

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    plt.ion()

    for row in rows:
        timestamp, sensor, rx, ry, rz, x, y, z = row
        pose_buffers[sensor].append((np.array([x, y, z]), np.array([rx, ry, rz])))

        ax.cla()

        # Fixed plot limits
        ax.set_xlim(center[0] - radius, center[0] + radius)
        ax.set_ylim(center[1] - radius, center[1] + radius)
        ax.set_zlim(center[2] - radius, center[2] + radius)
        ax.set_box_aspect([1, 1, 1])
        ax.set_autoscale_on(False)

        # Draw trajectories and orientations
        for sensor_name, poses in pose_buffers.items():
            if not poses:
                continue

            # Trajectory
            traj = np.array([p[0] for p in poses])
            ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], marker='o', linestyle='-', label=sensor_name)

            # Orientation axes for most recent pose
            pos, rvec = poses[-1]
            rot = R.from_rotvec(rvec)
            R_mat = rot.as_matrix()

            # Camera axes in world coordinates
            axes = R_mat * AXIS_LENGTH
            origin = pos[:, np.newaxis]

            # X axis (red)
            ax.plot([origin[0,0], origin[0,0] + axes[0,0]],
                    [origin[1,0], origin[1,0] + axes[1,0]],
                    [origin[2,0], origin[2,0] + axes[2,0]], color='r')

            # Y axis (green)
            ax.plot([origin[0,0], origin[0,0] + axes[0,1]],
                    [origin[1,0], origin[1,0] + axes[1,1]],
                    [origin[2,0], origin[2,0] + axes[2,1]], color='g')

            # Z axis (blue)
            ax.plot([origin[0,0], origin[0,0] + axes[0,2]],
                    [origin[1,0], origin[1,0] + axes[1,2]],
                    [origin[2,0], origin[2,0] + axes[2,2]], color='b')

        ax.set_title("Initial Camera Poses with Orientation")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.legend(loc="upper right")

        plt.draw()
        plt.pause(0.005)

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