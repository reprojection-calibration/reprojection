INSERT INTO workflow_steps(workflow_id, type, step_id)
VALUES (?, ?, ?)
ON CONFLICT(workflow_id, type)
    DO UPDATE SET step_id = excluded.step_id;