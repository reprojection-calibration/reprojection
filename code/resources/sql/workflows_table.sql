CREATE TABLE IF NOT EXISTS workflows
(
    id                    INTEGER PRIMARY KEY,
    type                  TEXT NOT NULL CHECK ( type IN ('cam', 'cam_imu', 'multi_cam')),
    asset_group_signature TEXT NOT NULL,

    FOREIGN KEY (asset_group_signature) REFERENCES asset_groups (signature) ON DELETE CASCADE,
    UNIQUE (type, asset_group_signature)
);