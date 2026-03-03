SELECT step_name,
       sensor_name,
       timestamp_ns,
       rx,
       ry,
       rz,
       x,
       y,
       z
FROM poses
ORDER BY timestamp_ns ASC;