CREATE TABLE IF NOT EXISTS imu_data
(
    sensor_name  TEXT    NOT NULL,
    timestamp_ns INTEGER NOT NULL,
    omega_x      REAL    NOT NULL,
    omega_y      REAL    NOT NULL,
    omega_z      REAL    NOT NULL,
    ax           REAL    NOT NULL,
    ay           REAL    NOT NULL,
    az           REAL    NOT NULL,
    PRIMARY KEY (timestamp_ns, sensor_name)
);