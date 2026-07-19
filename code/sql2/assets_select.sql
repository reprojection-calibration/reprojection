SELECT id, sensor_name
FROM assets
WHERE type = ?
  AND "index" = ?;