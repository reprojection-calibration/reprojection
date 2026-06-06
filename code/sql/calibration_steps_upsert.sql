INSERT INTO calibration_steps (step_name, entity_id, cache_key)
VALUES (?, ?, ?)
ON CONFLICT(step_name, entity_id) DO UPDATE SET cache_key  = excluded.cache_key,
                                                created_at = CURRENT_TIMESTAMP;