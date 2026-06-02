SELECT t0_ns,
       delta_t_ns
FROM spline_time_handler
WHERE step_name = ?
  AND sensor_name = ?