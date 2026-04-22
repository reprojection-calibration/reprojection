INSERT INTO calibration_steps (step_name, sensor_name, cache_key)
VALUES (?, ?, ?)
ON CONFLICT(step_name, sensor_name) DO UPDATE SET cache_key  = excluded.cache_key,
                                                  created_at = CURRENT_TIMESTAMP;