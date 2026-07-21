INSERT INTO recordings(name, hash)
VALUES (?, ?)
RETURNING id;