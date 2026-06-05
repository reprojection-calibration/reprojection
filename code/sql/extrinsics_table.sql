CREATE TABLE IF NOT EXISTS extrinsics
(
    step_name   TEXT NOT NULL,
    entity_id TEXT NOT NULL,
    rx          REAL NOT NULL,
    ry          REAL NOT NULL,
    rz          REAL NOT NULL,
    x           REAL NOT NULL,
    y           REAL NOT NULL,
    z           REAL NOT NULL,
    PRIMARY KEY (step_name, entity_id),
    FOREIGN KEY (step_name, entity_id) REFERENCES calibration_steps ON DELETE CASCADE
);