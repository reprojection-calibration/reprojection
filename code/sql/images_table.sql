CREATE TABLE IF NOT EXISTS images
(
    step_id      INTEGER NOT NULL,
    asset_id     INTEGER NOT NULL,
    timestamp_ns INTEGER NOT NULL,
    data         BLOB,

    FOREIGN KEY (step_id) REFERENCES steps (id) ON DELETE CASCADE,
    FOREIGN KEY (asset_id) REFERENCES assets (id) ON DELETE CASCADE,
    PRIMARY KEY (step_id, asset_id, timestamp_ns)
);