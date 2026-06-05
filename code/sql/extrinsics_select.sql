SELECT rx,
       ry,
       rz,
       x,
       y,
       z
FROM extrinsics
WHERE step_name = ?
  AND entity_id = ?