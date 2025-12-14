import sqlite3
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

DB_PATH = "/home/stable-genius-gram/data/cvg.cit.tum.de_visual-inertial-dataset/dataset-calib-imu4_512_16.db3"
AXIS_LENGTH = 0.25
GRID_SIZE = 10
GRID_STEP = 1

paused = False   # ⏸ global pause flag


def toggle_pause(vis):
    global paused
    paused = not paused
    print("Paused" if paused else "Playing")
    return False


def load_poses(db_path):
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute(
        "SELECT timestamp_ns, sensor_name, rx, ry, rz, x, y, z "
        "FROM initial_camera_poses ORDER BY timestamp_ns ASC"
    )
    rows = cur.fetchall()
    conn.close()
    return rows


def create_grid_lines(grid_size=GRID_SIZE, step=GRID_STEP):
    points, lines = [], []
    idx = 0

    for i in range(-grid_size, grid_size + 1):
        points.append([i * step, -grid_size * step, 0])
        points.append([i * step, grid_size * step, 0])
        lines.append([idx, idx + 1])
        idx += 2

        points.append([-grid_size * step, i * step, 0])
        points.append([grid_size * step, i * step, 0])
        lines.append([idx, idx + 1])
        idx += 2

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector([[0.7, 0.7, 0.7]] * len(lines))
    return line_set


def run_open3d_viz(rows):
    global paused

    # 🔑 Use key-callback visualizer
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window("Open3D Pose Visualizer", 1280, 720)

    # Space bar = 32
    vis.register_key_callback(32, toggle_pause)

    sensors = sorted(set(r[1] for r in rows))
    traj_geoms = {}
    coord_frames = {}
    previous_transforms = {s: np.eye(4) for s in sensors}
    trajectories = {s: [] for s in sensors}

    for s in sensors:
        traj_geoms[s] = o3d.geometry.LineSet()
        vis.add_geometry(traj_geoms[s])

        coord_frames[s] = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=AXIS_LENGTH
        )
        vis.add_geometry(coord_frames[s])

    vis.add_geometry(
        o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    )
    vis.add_geometry(create_grid_lines())

    row_idx = 0
    while True:
        if not paused and row_idx < len(rows):
            _, sensor, rx, ry, rz, x, y, z = rows[row_idx]
            row_idx += 1

            pos = np.array([x, y, z])
            rvec = np.array([rx, ry, rz])

            # --- Trajectory ---
            trajectories[sensor].append(pos)
            points = np.array(trajectories[sensor])

            if len(points) > 1:
                lines = np.vstack(
                    [np.arange(len(points) - 1), np.arange(1, len(points))]
                ).T
                traj_geoms[sensor].points = o3d.utility.Vector3dVector(points)
                traj_geoms[sensor].lines = o3d.utility.Vector2iVector(lines)

                color = [0, 0.4, 0.8] if sensor == sensors[0] else [1, 0.5, 0]
                traj_geoms[sensor].colors = o3d.utility.Vector3dVector(
                    [color] * len(lines)
                )
                vis.update_geometry(traj_geoms[sensor])

            # --- Camera pose ---
            rot_mat = R.from_rotvec(rvec).as_matrix()
            T = np.eye(4)
            T[:3, :3] = rot_mat
            T[:3, 3] = pos

            delta = T @ np.linalg.inv(previous_transforms[sensor])
            coord_frames[sensor].transform(delta)
            previous_transforms[sensor] = T

            vis.update_geometry(coord_frames[sensor])

        if not vis.poll_events():
            break
        vis.update_renderer()

    print("Sequence finished. Manual inspection mode enabled.")
    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    rows = load_poses(DB_PATH)
    if rows:
        run_open3d_viz(rows)