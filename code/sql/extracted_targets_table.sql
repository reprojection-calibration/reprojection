CREATE TABLE IF NOT EXISTS extracted_targets
(
    step_name    TEXT    NOT NULL CHECK ( step_name IN ('feature_extraction')),
    sensor_name  TEXT    NOT NULL,
    timestamp_ns INTEGER NOT NULL,
    data         BLOB    NOT NULL,
    PRIMARY KEY (sensor_name, timestamp_ns),
    FOREIGN KEY (step_name, sensor_name) REFERENCES calibration_steps (step_name, sensor_name) ON DELETE CASCADE,
    FOREIGN KEY (sensor_name, timestamp_ns) REFERENCES images (sensor_name, timestamp_ns)
);