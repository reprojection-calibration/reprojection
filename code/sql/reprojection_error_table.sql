CREATE TABLE IF NOT EXISTS reprojection_error
(
    step_name    TEXT    NOT NULL,
    sensor_name  TEXT    NOT NULL,
    timestamp_ns INTEGER NOT NULL,
    data         BLOB    NOT NULL,
    PRIMARY KEY (step_name, sensor_name, timestamp_ns),
    FOREIGN KEY (step_name, sensor_name, timestamp_ns)
        REFERENCES poses (step_name, sensor_name, timestamp_ns)
);