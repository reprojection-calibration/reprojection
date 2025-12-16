import sqlite3
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import extracted_target_pb2  # generated from your .proto

DB_PATH = "/home/stable-genius-gram/data/cvg.cit.tum.de_visual-inertial-dataset/dataset-calib-imu4_512_16.db3"

AXIS_LENGTH = 0.25
GRID_SIZE = 10
GRID_STEP = 1

paused = False


def toggle_pause(vis):
    global paused
    paused = not paused
    print("Paused" if paused else "Playing")
    return False


def load_poses(db_path, pose_type):
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute(
        "SELECT timestamp_ns, sensor_name, type, rx, ry, rz, x, y, z "
        "FROM camera_poses WHERE type = ? ORDER BY timestamp_ns ASC",
        (pose_type,)
    )
    rows = cur.fetchall()
    conn.close()
    return rows


def load_extracted_targets(db_path):
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute(
        "SELECT timestamp_ns, sensor_name, data "
        "FROM extracted_targets ORDER BY timestamp_ns ASC"
    )

    targets = {}
    for ts, sensor, blob in cur.fetchall():
        msg = extracted_target_pb2.ExtractedTargetProto()
        msg.ParseFromString(blob)

        rows = msg.bundle.point_rows
        data = np.array(msg.bundle.point_data, dtype=np.float64)

        if rows == 0 or data.size == 0:
            points = np.empty((0, 3))
        else:
            points = data.reshape(3, rows).transpose()  # Eigen::MatrixX3d

        targets.setdefault(sensor, {})[ts] = points

    conn.close()
    return targets


def create_grid_lines():
    points, lines = [], []
    idx = 0

    for i in range(-GRID_SIZE, GRID_SIZE + 1):
        points.append([i * GRID_STEP, -GRID_SIZE * GRID_STEP, 0])
        points.append([i * GRID_STEP, GRID_SIZE * GRID_STEP, 0])
        lines.append([idx, idx + 1])
        idx += 2

        points.append([-GRID_SIZE * GRID_STEP, i * GRID_STEP, 0])
        points.append([GRID_SIZE * GRID_STEP, i * GRID_STEP, 0])
        lines.append([idx, idx + 1])
        idx += 2

    grid = o3d.geometry.LineSet()
    grid.points = o3d.utility.Vector3dVector(points)
    grid.lines = o3d.utility.Vector2iVector(lines)
    grid.colors = o3d.utility.Vector3dVector([[0.7, 0.7, 0.7]] * len(lines))
    return grid


def run_open3d_viz(pose_rows, target_data):
    global paused

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window("Open3D Pose + Target Visualizer", 1280, 720)
    vis.register_key_callback(32, toggle_pause)

    sensors = sorted(set(r[1] for r in pose_rows))

    traj_geoms = {}
    coord_frames = {}
    pointclouds = {}

    previous_transforms = {s: np.eye(4) for s in sensors}
    trajectories = {s: [] for s in sensors}

    for s in sensors:
        traj_geoms[s] = o3d.geometry.LineSet()
        vis.add_geometry(traj_geoms[s])

        coord_frames[s] = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=AXIS_LENGTH
        )
        vis.add_geometry(coord_frames[s])

        pc = o3d.geometry.PointCloud()
        pc.paint_uniform_color([1, 0, 0])  # detected targets in red
        pointclouds[s] = pc
        vis.add_geometry(pc)

    vis.add_geometry(
        o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    )
    vis.add_geometry(create_grid_lines())

    row_idx = 0
    while True:
        if not paused and row_idx < len(pose_rows):
            ts, sensor, type, rx, ry, rz, x, y, z = pose_rows[row_idx]
            row_idx += 1

            pos = np.array([x, y, z])
            rvec = np.array([rx, ry, rz])

            # ---- Trajectory ----
            trajectories[sensor].append(pos)
            pts = np.array(trajectories[sensor])

            if len(pts) > 1:
                lines = np.vstack(
                    [np.arange(len(pts) - 1), np.arange(1, len(pts))]
                ).T
                traj_geoms[sensor].points = o3d.utility.Vector3dVector(pts)
                traj_geoms[sensor].lines = o3d.utility.Vector2iVector(lines)
                traj_geoms[sensor].colors = o3d.utility.Vector3dVector(
                    [[0, 0.4, 0.8]] * len(lines)
                )
                vis.update_geometry(traj_geoms[sensor])

            # ---- Camera pose ----
            rot = R.from_rotvec(rvec).as_matrix()
            T = np.eye(4)
            T[:3, :3] = rot
            T[:3, 3] = pos

            delta = T @ np.linalg.inv(previous_transforms[sensor])
            coord_frames[sensor].transform(delta)
            previous_transforms[sensor] = T
            vis.update_geometry(coord_frames[sensor])

            # ---- Extracted targets ----
            if sensor in target_data and ts in target_data[sensor]:
                pts_world = target_data[sensor][ts]

                pointclouds[sensor].points = o3d.utility.Vector3dVector(
                    pts_world
                )
                vis.update_geometry(pointclouds[sensor])

        if not vis.poll_events():
            break
        vis.update_renderer()

    print("Sequence finished. Manual inspection mode enabled.")
    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    poses = load_poses(DB_PATH, 'initial')
    targets = load_extracted_targets(DB_PATH)

    if poses:
        run_open3d_viz(poses, targets)