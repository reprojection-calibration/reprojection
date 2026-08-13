SELECT timestamp_ns, rx, ry, rz, x,y,z
FROM camera_poses
WHERE step_id = ?
  AND asset_id = ?;