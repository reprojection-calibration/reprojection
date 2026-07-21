SELECT target_type, height, width, unit_dimension, asymmetric
FROM target_info
WHERE step_id = ?
  AND asset_id = ?;