SELECT idx, _0, _1, _2, _3, _4, _5
FROM control_points
WHERE step_id = ?
  AND asset_id = ?
ORDER BY idx ASC;