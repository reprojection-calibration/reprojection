SELECT spline_type, t0_ns, delta_t_ns
FROM spline_info
WHERE step_id = ?
  AND asset_id = ?;