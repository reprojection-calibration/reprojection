SELECT model,
       width,
       height,
       intrinsics
FROM intrinsics
WHERE step_name = ? AND sensor_name = ?