INSERT INTO steps (recording_id, run_id, type, cache_key)
VALUES (?, ?, ?, ?)
RETURNING id;