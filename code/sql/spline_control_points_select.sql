SELECT id,
       rx,
       ry,
       rz,
       x,
       y,
       z
FROM spline_control_points
WHERE step_name = ?
  AND sensor_name = ?