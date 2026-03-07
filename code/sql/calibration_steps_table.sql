CREATE TABLE IF NOT EXISTS calibration_steps
(
    step_name   TEXT CHECK ( step_name IN ('linear_pose_initialization',
                                           'camera_nonlinear_refinement',
                                           'spline_interpolation',
                                           'spline_nonlinear_refinement')),
    sensor_name TEXT     NOT NULL,
    cache_key   TEXT     NOT NULL,
    created_at  DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    PRIMARY KEY (step_name, sensor_name),
    FOREIGN KEY (sensor_name)
        REFERENCES camera_info (sensor_name)
);