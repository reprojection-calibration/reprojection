SELECT camera_model, height, width
FROM camera_info
WHERE step_id = ?
  AND asset_id = ?;