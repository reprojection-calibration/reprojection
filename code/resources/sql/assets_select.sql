SELECT id, name
FROM assets
WHERE type = ?
  AND "index" = ?;