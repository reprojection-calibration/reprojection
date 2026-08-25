CREATE TABLE IF NOT EXISTS workflow_steps
(
    workflow_id           INTEGER NOT NULL,
    type                  TEXT    NOT NULL,
    step_id               INTEGER NOT NULL,
    asset_group_signature TEXT    NOT NULL,

    FOREIGN KEY (workflow_id) REFERENCES workflows (id) ON DELETE CASCADE,
    FOREIGN KEY (step_id, type) REFERENCES steps (id, type) ON DELETE CASCADE,
    FOREIGN KEY (asset_group_signature) REFERENCES asset_groups (signature) ON DELETE CASCADE,
    PRIMARY KEY (workflow_id, type)
);