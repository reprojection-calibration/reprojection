CREATE TABLE IF NOT EXISTS imu_errors
(
    step_id        INTEGER NOT NULL,
    source_step_id INTEGER NOT NULL,
    asset_id       INTEGER NOT NULL,
    timestamp_ns   INTEGER NOT NULL,
    omega_x        REAL    NOT NULL,
    omega_y        REAL    NOT NULL,
    omega_z        REAL    NOT NULL,
    ax             REAL    NOT NULL,
    ay             REAL    NOT NULL,
    az             REAL    NOT NULL,


    FOREIGN KEY (step_id) REFERENCES steps (id) ON DELETE CASCADE,
    FOREIGN KEY (source_step_id, asset_id, timestamp_ns) REFERENCES imu_data (step_id, asset_id, timestamp_ns) ON DELETE CASCADE,
    PRIMARY KEY (step_id, asset_id, timestamp_ns)
);