CREATE TABLE IF NOT EXISTS imu_errors
(
    step_name     TEXT    NOT NULL,
    entity_id     TEXT    NOT NULL,
    imu_name      TEXT    NOT NULL,
    timestamp_ns  INTEGER NOT NULL,
    delta_omega_x REAL    NOT NULL,
    delta_omega_y REAL    NOT NULL,
    delta_omega_z REAL    NOT NULL,
    delta_ax      REAL    NOT NULL,
    delta_ay      REAL    NOT NULL,
    delta_az      REAL    NOT NULL,

    FOREIGN KEY (step_name, entity_id) REFERENCES calibration_steps (step_name, entity_id) ON DELETE CASCADE,
    FOREIGN KEY (imu_name, timestamp_ns) REFERENCES imu_data (sensor_name, timestamp_ns),
    PRIMARY KEY (step_name, entity_id, timestamp_ns)
);