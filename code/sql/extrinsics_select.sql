SELECT frame_a,
       frame_b,
       rx,
       ry,
       rz,
       x,
       y,
       z
FROM extrinsics
WHERE step_name = ?
  AND entity_id = ?