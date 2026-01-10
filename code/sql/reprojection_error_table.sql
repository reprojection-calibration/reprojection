CREATE TABLE IF NOT EXISTS reprojection_error
(
    timestamp_ns INTEGER NOT NULL,
    sensor_name  TEXT    NOT NULL,
    type         TEXT    NOT NULL,
    data         BLOB    NOT NULL,
    PRIMARY KEY (timestamp_ns, sensor_name, type),
    FOREIGN KEY (timestamp_ns, sensor_name, type)
        REFERENCES camera_poses (timestamp_ns, sensor_name, type)
);
