SELECT timestamp_ns, data
FROM images
WHERE step_id = ?
  AND asset_id = ?;