INSERT INTO workflow_assets (workflow_id, asset_id)
VALUES (?, ?)
ON CONFLICT(workflow_id, asset_id) DO NOTHING;