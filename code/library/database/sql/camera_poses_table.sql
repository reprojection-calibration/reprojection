CREATE TABLE IF NOT EXISTS camera_poses
(
    timestamp_ns INTEGER NOT NULL,
    sensor_name  TEXT    NOT NULL,
    type         TEXT    NOT NULL,
    rx           REAL    NOT NULL,
    ry           REAL    NOT NULL,
    rz           REAL    NOT NULL,
    x            REAL    NOT NULL,
    y            REAL    NOT NULL,
    z            REAL    NOT NULL,
    PRIMARY KEY (timestamp_ns, sensor_name, type),
    FOREIGN KEY (timestamp_ns, sensor_name)
        REFERENCES extracted_targets (timestamp_ns, sensor_name)
);

