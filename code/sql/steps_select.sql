SELECT id
FROM steps
WHERE type = ?
  AND cache_key = ?;