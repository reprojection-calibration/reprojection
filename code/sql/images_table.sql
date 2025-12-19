CREATE TABLE IF NOT EXISTS images
(
    timestamp_ns INTEGER NOT NULL,
    sensor_name  TEXT    NOT NULL,
    data         BLOB    NULL,
    PRIMARY KEY (timestamp_ns, sensor_name)
);