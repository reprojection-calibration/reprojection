SELECT id, cache_key
FROM steps
WHERE run_id = ?
  AND type = ?;