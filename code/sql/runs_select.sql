SELECT id, config
FROM runs
WHERE recording_id = ?
  AND config_hash = ?;