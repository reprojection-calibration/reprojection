SELECT timestamp_ns,
       rx,
       ry,
       rz,
       x,
       y,
       z
FROM poses
WHERE step_name = ?
  AND sensor_name = ?
ORDER BY timestamp_ns ASC;