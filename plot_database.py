import sqlite3
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

DB_PATH = "/home/stable-genius-gram/data/cvg.cit.tum.de_visual-inertial-dataset/dataset-calib-imu4_512_16.db3"
AXIS_LENGTH = 0.25
GRID_SIZE = 10       # Number of grid lines in one direction
GRID_STEP = 1      # Spacing between lines

def load_poses(db_path):
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute(
        "SELECT timestamp_ns, sensor_name, rx, ry, rz, x, y, z FROM initial_camera_poses ORDER BY timestamp_ns ASC"
    )
    rows = cur.fetchall()
    conn.close()
    return rows

def create_grid_lines(grid_size=GRID_SIZE, step=GRID_STEP):
    """Create a flat grid on the XY plane at z=0"""
    points = []
    lines = []

    idx = 0
    for i in range(-grid_size, grid_size+1):
        # Line along X axis
        points.append([i*step, -grid_size*step, 0])
        points.append([i*step, grid_size*step, 0])
        lines.append([idx, idx+1])
        idx += 2
        # Line along Y axis
        points.append([-grid_size*step, i*step, 0])
        points.append([grid_size*step, i*step, 0])
        lines.append([idx, idx+1])
        idx += 2

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector([[0.7, 0.7, 0.7] for _ in lines])
    return line_set

def run_open3d_viz(rows):
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Open3D Pose Visualizer", width=1280, height=720)

    sensors = list(sorted(set(r[1] for r in rows)))
    traj_geoms = {}
    coord_frames = {}
    previous_transforms = {s: np.eye(4) for s in sensors}
    trajectories = {s: [] for s in sensors}

    # Setup geometries
    for s in sensors:
        # 1. Path LineSet
        ls = o3d.geometry.LineSet()
        traj_geoms[s] = ls
        vis.add_geometry(ls)

        # 2. Coordinate Frame (Camera Axis)
        cf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=AXIS_LENGTH)
        coord_frames[s] = cf
        vis.add_geometry(cf)

    # Reference World Frame + Grid
    world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    vis.add_geometry(world_frame)

    grid = create_grid_lines()
    vis.add_geometry(grid)

    for idx, row in enumerate(rows):
        _, sensor, rx, ry, rz, x, y, z = row
        pos = np.array([x, y, z])
        rvec = np.array([rx, ry, rz])

        # --- Update Trajectory ---
        trajectories[sensor].append(pos)
        points = np.array(trajectories[sensor])

        if len(points) > 1:
            lines = np.vstack([np.arange(len(points)-1), np.arange(1, len(points))]).T
            traj_geoms[sensor].points = o3d.utility.Vector3dVector(points)
            traj_geoms[sensor].lines = o3d.utility.Vector2iVector(lines)

            color = [0, 0.4, 0.8] if sensor == sensors[0] else [1, 0.5, 0]
            traj_geoms[sensor].colors = o3d.utility.Vector3dVector([color for _ in range(len(lines))])
            vis.update_geometry(traj_geoms[sensor])

        # --- Update Camera Pose (Absolute Positioning) ---
        rot_mat = R.from_rotvec(rvec).as_matrix()
        new_transform_4x4 = np.eye(4)
        new_transform_4x4[:3, :3] = rot_mat
        new_transform_4x4[:3, 3] = pos

        undo_transform = np.linalg.inv(previous_transforms[sensor])
        absolute_jump = new_transform_4x4 @ undo_transform
        coord_frames[sensor].transform(absolute_jump)
        previous_transforms[sensor] = new_transform_4x4

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
