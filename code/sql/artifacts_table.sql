CREATE TABLE IF NOT EXISTS artifacts
(
    id      INTEGER PRIMARY KEY,
    step_id INTEGER NOT NULL,
    type    TEXT    NOT NULL CHECK (
        type IN ('images', 'imu_data', 'extracted_targets')
        ),

    FOREIGN KEY (step_id) REFERENCES steps (id) ON DELETE CASCADE,
    UNIQUE (step_id, type)
);