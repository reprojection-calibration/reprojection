CREATE TABLE IF NOT EXISTS external_poses
(
    timestamp_ns INTEGER NOT NULL,
    sensor_name  TEXT    NOT NULL,
    rx           REAL    NOT NULL,
    ry           REAL    NOT NULL,
    rz           REAL    NOT NULL,
    x            REAL    NOT NULL,
    y            REAL    NOT NULL,
    z            REAL    NOT NULL,
    PRIMARY KEY (timestamp_ns, sensor_name)
);

