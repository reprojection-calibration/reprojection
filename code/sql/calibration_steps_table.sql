CREATE TABLE IF NOT EXISTS calibration_steps
(
    step_name   TEXT CHECK ( step_name IN ('camera_nonlinear_refinement',
                                           'feature_extraction',
                                           'intrinsic_initialization',
                                           'linear_pose_initialization',
                                           'spline_interpolation',
                                           'spline_nonlinear_refinement')),
    sensor_name TEXT     NOT NULL,
    cache_key   TEXT     NOT NULL,
    created_at  DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    PRIMARY KEY (step_name, sensor_name)
);