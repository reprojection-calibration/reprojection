CREATE TABLE IF NOT EXISTS imu_data
(
    step_name    TEXT    NOT NULL CHECK ( step_name IN ('imu_data_loading')),
    sensor_name  TEXT    NOT NULL,
    timestamp_ns INTEGER NOT NULL,
    omega_x      REAL    NOT NULL,
    omega_y      REAL    NOT NULL,
    omega_z      REAL    NOT NULL,
    ax           REAL    NOT NULL,
    ay           REAL    NOT NULL,
    az           REAL    NOT NULL,

    FOREIGN KEY (step_name, sensor_name) REFERENCES calibration_steps (step_name, entity_id) ON DELETE CASCADE,
    PRIMARY KEY (sensor_name, timestamp_ns)
);