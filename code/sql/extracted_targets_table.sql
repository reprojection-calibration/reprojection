CREATE TABLE IF NOT EXISTS extracted_targets
(
    sensor_name  TEXT    NOT NULL,
    timestamp_ns INTEGER NOT NULL,
    data         BLOB    NOT NULL,
    PRIMARY KEY (sensor_name, timestamp_ns),
    FOREIGN KEY (sensor_name, timestamp_ns)
        REFERENCES images (sensor_name, timestamp_ns)
);