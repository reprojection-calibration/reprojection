INSERT INTO steps (id, recording_id, run_id, type, cache_key)
VALUES (?, ?, ?, ?, ?)
RETURNING id;