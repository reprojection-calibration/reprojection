CREATE TABLE IF NOT EXISTS extrinsics
(
    step_name TEXT NOT NULL,
    entity_id TEXT NOT NULL,
    frame_a   TEXT NOT NULL,
    frame_b   TEXT NOT NULL,
    rx        REAL NOT NULL,
    ry        REAL NOT NULL,
    rz        REAL NOT NULL,
    x         REAL NOT NULL,
    y         REAL NOT NULL,
    z         REAL NOT NULL,

    FOREIGN KEY (step_name, entity_id) REFERENCES calibration_steps ON DELETE CASCADE,
    FOREIGN KEY (frame_a) REFERENCES entity (entity_id),
    FOREIGN KEY (frame_b) REFERENCES entity (entity_id),
    PRIMARY KEY (step_name, entity_id)
);