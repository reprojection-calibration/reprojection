CREATE TABLE IF NOT EXISTS images
(
    sensor_name  TEXT    NOT NULL,
    timestamp_ns INTEGER NOT NULL,
    data         BLOB    NULL,
    PRIMARY KEY (sensor_name, timestamp_ns),
    FOREIGN KEY (sensor_name)
        REFERENCES camera_info (sensor_name)
);