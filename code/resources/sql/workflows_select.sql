SELECT id
FROM workflows
WHERE type = ?
  AND asset_group_signature = ?;