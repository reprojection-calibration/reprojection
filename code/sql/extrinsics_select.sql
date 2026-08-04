SELECT rx, ry, rz, x, y, z
FROM extrinsics
WHERE step_id = ?
  AND asset_a_id = ?
  AND asset_b_id = ?;