CREATE TABLE IF NOT EXISTS imu_error
(
    step_name     TEXT    NOT NULL,
    sensor_name   TEXT    NOT NULL,
    timestamp_ns  INTEGER NOT NULL,
    delta_omega_x REAL    NOT NULL,
    delta_omega_y REAL    NOT NULL,
    delta_omega_z REAL    NOT NULL,
    delta_ax      REAL    NOT NULL,
    delta_ay      REAL    NOT NULL,
    delta_az      REAL    NOT NULL,
    PRIMARY KEY (step_name, sensor_name, timestamp_ns),
    FOREIGN KEY (step_name, sensor_name) REFERENCES calibration_steps ON DELETE CASCADE,
    FOREIGN KEY (sensor_name, timestamp_ns) REFERENCES imu_data
);