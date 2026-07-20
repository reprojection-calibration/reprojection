INSERT INTO runs(recording_id, config_hash, config)
VALUES (?, ?, ?)
RETURNING id;