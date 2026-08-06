CREATE TABLE IF NOT EXISTS workflow_steps
(
    workflow_id INTEGER NOT NULL,
    step_id    INTEGER NOT NULL,

    FOREIGN KEY (workflow_id) REFERENCES workflows (id) ON DELETE CASCADE,
    FOREIGN KEY (step_id) REFERENCES steps (id),
    PRIMARY KEY (workflow_id, step_id)
);