SELECT camera_model, data
FROM intrinsics
WHERE step_id = ?
  AND asset_id = ?;