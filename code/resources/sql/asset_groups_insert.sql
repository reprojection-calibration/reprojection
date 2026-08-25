INSERT INTO asset_groups (signature)
VALUES (?)
ON CONFLICT(signature) DO NOTHING;