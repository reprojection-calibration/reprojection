CREATE TABLE IF NOT EXISTS extracted_targets
(
    timestamp_ns INTEGER NOT NULL,
    sensor_name  TEXT    NOT NULL,
    data         BLOB    NOT NULL,
    PRIMARY KEY (timestamp_ns, sensor_name),
    FOREIGN KEY (timestamp_ns, sensor_name)
        REFERENCES images (timestamp_ns, sensor_name)
);