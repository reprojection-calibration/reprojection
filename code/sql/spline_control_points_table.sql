CREATE TABLE IF NOT EXISTS spline_control_points
(
    step_name   TEXT    NOT NULL,
    sensor_name TEXT    NOT NULL,
    id       INTEGER NOT NUll,
    rx          REAL    NOT NULL,
    ry          REAL    NOT NULL,
    rz          REAL    NOT NULL,
    x           REAL    NOT NULL,
    y           REAL    NOT NULL,
    z           REAL    NOT NULL,
    PRIMARY KEY (step_name, sensor_name, id),
    FOREIGN KEY (step_name, sensor_name) REFERENCES calibration_steps ON DELETE CASCADE
);