SELECT timestamp_ns, data
FROM extracted_targets
WHERE step_id = ?
  AND asset_id = ?;