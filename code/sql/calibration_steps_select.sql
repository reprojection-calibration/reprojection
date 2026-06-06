SELECT cache_key
FROM calibration_steps
WHERE step_name = ?
  AND entity_id = ?