SELECT ax,
       ay,
       az
FROM gravity
WHERE step_name = ?
  AND entity_id = ?