CREATE TABLE IF NOT EXISTS poses
(
    step_name    TEXT    NOT NULL,
    timestamp_ns INTEGER NOT NULL,
    sensor_name  TEXT    NOT NULL,
    rx           REAL    NOT NULL,
    ry           REAL    NOT NULL,
    rz           REAL    NOT NULL,
    x            REAL    NOT NULL,
    y            REAL    NOT NULL,
    z            REAL    NOT NULL,
    PRIMARY KEY (step_name, timestamp_ns, sensor_name),
    FOREIGN KEY (step_name) REFERENCES calibration_steps (step_name)
);