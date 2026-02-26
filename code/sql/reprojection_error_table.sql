CREATE TABLE IF NOT EXISTS reprojection_error
(
    timestamp_ns INTEGER NOT NULL,
    sensor_name  TEXT    NOT NULL,
    step_name    TEXT    NOT NULL,
    data         BLOB    NOT NULL,
    PRIMARY KEY (step_name, timestamp_ns, sensor_name),
    FOREIGN KEY (step_name, timestamp_ns, sensor_name)
        REFERENCES poses (step_name, timestamp_ns, sensor_name)
);