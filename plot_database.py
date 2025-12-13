import sqlite3
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

DB_PATH = "/home/stable-genius-gram/data/cvg.cit.tum.de_visual-inertial-dataset/dataset-calib-imu4_512_16.db3"
AXIS_LENGTH = 0.25

def load_poses(db_path):
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute("SELECT timestamp_ns, sensor_name, rx, ry, rz, x, y, z FROM initial_camera_poses ORDER BY timestamp_ns ASC")
    rows = cur.fetchall()
    conn.close()
    return rows

def run_open3d_viz(rows):
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Open3D Pose Visualizer", width=1280, height=720)

    sensors = list(sorted(set(r[1] for r in rows)))
    traj_geoms = {}
    coord_frames = {}
    # We need to store the PREVIOUS transformation matrix for each sensor
    # Initialize all previous transformations to the Identity matrix (4x4)
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

    # Reference World Grid
    world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    vis.add_geometry(world_frame)

    for idx, row in enumerate(rows):
        _, sensor, rx, ry, rz, x, y, z = row
        pos = np.array([x, y, z])
        rvec = np.array([rx, ry, rz])

        # --- Update Trajectory ---
        # (This part is identical and works correctly)
        trajectories[sensor].append(pos)
        points = np.array(trajectories[sensor])

        if len(points) > 1:
            lines = np.vstack([np.arange(len(points)-1), np.arange(1, len(points))]).T
            traj_geoms[sensor].points = o3d.utility.Vector3dVector(points)
            traj_geoms[sensor].lines = o3d.utility.Vector2iVector(lines)

            color = [0, 0.4, 0.8] if sensor == sensors[0] else [1, 0.5, 0]
            traj_geoms[sensor].paint_uniform_color(color)
            vis.update_geometry(traj_geoms[sensor])

        # --- Update Camera Pose (Absolute Positioning) ---

        # 1. Calculate the NEW 4x4 Transformation Matrix
        rot_mat = R.from_rotvec(rvec).as_matrix()
        new_transform_4x4 = np.eye(4)
        new_transform_4x4[:3, :3] = rot_mat
        new_transform_4x4[:3, 3] = pos

        # 2. Calculate the "undo" matrix: Inverse of the previous transformation
        undo_transform = np.linalg.inv(previous_transforms[sensor])

        # 3. Calculate the matrix that jumps from T_old to T_new: T_jump = T_new @ T_old_inverse
        # This is the relative transformation needed to jump to the new absolute pose.
        # This is equivalent to T_jump = T_new @ T_old_inverse
        absolute_jump = new_transform_4x4 @ undo_transform

        # 4. Apply the ABSOLUTE JUMP using the reliable transform() method
        coord_frames[sensor].transform(absolute_jump)

        # 5. Store the NEW transformation for the next frame's "undo" calculation
        previous_transforms[sensor] = new_transform_4x4

        vis.update_geometry(coord_frames[sensor])

        # --- Render ---
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