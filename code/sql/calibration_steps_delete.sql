DELETE
FROM calibration_steps
WHERE step_name = ?
  AND sensor_name = ?;