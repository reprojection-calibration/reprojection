CREATE TABLE IF NOT EXISTS images
(
    step_name    TEXT    NOT NULL CHECK ( step_name IN ('image_loading')),
    sensor_name  TEXT    NOT NULL,
    timestamp_ns INTEGER NOT NULL,
    data         BLOB    NULL,
    PRIMARY KEY (sensor_name, timestamp_ns),
    FOREIGN KEY (step_name, sensor_name) REFERENCES calibration_steps (step_name, sensor_name) ON DELETE CASCADE
);