INSERT INTO workflow_steps(workflow_id, step_id, type, asset_group_signature)
VALUES (?, ?, ?, ?)
ON CONFLICT(workflow_id, type, asset_group_signature)
    DO UPDATE SET step_id = excluded.step_id;