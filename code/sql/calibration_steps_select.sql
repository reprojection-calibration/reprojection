SELECT cache_key
FROM calibration_steps
WHERE step_name = ?
  AND sensor_name = ?
LIMIT 1;