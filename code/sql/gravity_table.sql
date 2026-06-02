CREATE TABLE IF NOT EXISTS gravity
(
    step_name   TEXT NOT NULL,
    sensor_name TEXT NOT NULL,
    ax           REAL NOT NULL,
    ay           REAL NOT NULL,
    az           REAL NOT NULL,
    PRIMARY KEY (step_name, sensor_name),
    FOREIGN KEY (step_name, sensor_name) REFERENCES calibration_steps ON DELETE CASCADE
);