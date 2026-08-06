CREATE TABLE IF NOT EXISTS workflow_assets
(
    workflow_id INTEGER NOT NULL,
    asset_id    INTEGER NOT NULL,

    FOREIGN KEY (workflow_id) REFERENCES workflows (id) ON DELETE CASCADE,
    FOREIGN KEY (asset_id) REFERENCES assets (id),
    PRIMARY KEY (workflow_id, asset_id)
);