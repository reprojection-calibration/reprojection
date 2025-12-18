SELECT timestamp_ns,
       sensor_name,
       type,
       rx,
       ry,
       rz,
       x,
       y,
       z
FROM camera_poses
WHERE type = ?
ORDER BY timestamp_ns ASC