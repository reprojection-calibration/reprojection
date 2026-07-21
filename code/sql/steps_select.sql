SELECT id, cache_key
FROM steps
WHERE recording_id IS ?
  AND run_id IS ?
  AND type = ?;