INSERT INTO steps(    id,    run_id,    type,    cache_key)
VALUES (?, ?, ?, ?)
RETURNING id;