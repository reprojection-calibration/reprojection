CREATE TABLE IF NOT EXISTS gravity
(
    step_name TEXT NOT NULL,
    entity_id TEXT NOT NULL,
    ax        REAL NOT NULL,
    ay        REAL NOT NULL,
    az        REAL NOT NULL,

    FOREIGN KEY (step_name, entity_id) REFERENCES calibration_steps ON DELETE CASCADE,
    PRIMARY KEY (step_name, entity_id)
);