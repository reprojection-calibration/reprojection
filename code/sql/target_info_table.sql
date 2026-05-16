CREATE TABLE IF NOT EXISTS target_info
(
    step_name      TEXT        NOT NULL CHECK ( step_name IN ('target_info')),
    sensor_name    TEXT UNIQUE NOT NULL,
    target_type    TEXT        NOT NULL CHECK ( target_type IN ('aprilgrid3', 'checkerboard', 'circle_grid')),
    height         INTEGER     NOT NULL,
    width          INTEGER     NOT NULL,
    unit_dimension REAL        NOT NULL,
    asymmetric     INTEGER     NOT NULL CHECK ( asymmetric IN (0, 1)),
    PRIMARY KEY (sensor_name, target_type),
    FOREIGN KEY (step_name, sensor_name) REFERENCES calibration_steps ON DELETE CASCADE
);