INSERT INTO workflows (type, asset_group_signature)
VALUES (?, ?)
RETURNING id;