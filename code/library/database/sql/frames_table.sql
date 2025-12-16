CREATE TABLE IF NOT EXISTS frames
(
    timestamp_ns INTEGER NOT NULL,
    sensor_name  TEXT    NOT NULL,
    PRIMARY KEY (timestamp_ns, sensor_name)
);