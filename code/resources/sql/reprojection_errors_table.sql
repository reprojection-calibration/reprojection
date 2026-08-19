CREATE TABLE IF NOT EXISTS reprojection_errors
(
    step_id        INTEGER NOT NULL,
    source_step_id INTEGER NOT NULL,
    asset_id       INTEGER NOT NULL,
    timestamp_ns   INTEGER NOT NULL,
    data           BLOB    NOT NULL,

    FOREIGN KEY (step_id) REFERENCES steps (id) ON DELETE CASCADE,
    FOREIGN KEY (source_step_id, asset_id, timestamp_ns) REFERENCES extracted_targets (step_id, asset_id, timestamp_ns) ON DELETE CASCADE,
    PRIMARY KEY (step_id, asset_id, timestamp_ns)
);