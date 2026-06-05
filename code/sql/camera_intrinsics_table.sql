CREATE TABLE IF NOT EXISTS camera_intrinsics
(
    step_name    TEXT NOT NULL,
    sensor_name  TEXT NOT NULL,
    camera_model TEXT NOT NULL,
    intrinsics   TEXT NOT NULL,

    FOREIGN KEY (step_name, sensor_name) REFERENCES calibration_steps (step_name, entity_id) ON DELETE CASCADE,
    FOREIGN KEY (sensor_name, camera_model) REFERENCES camera_info ON DELETE CASCADE,
    PRIMARY KEY (step_name, sensor_name)
);