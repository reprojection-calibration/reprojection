CREATE TABLE IF NOT EXISTS calibration_steps
(
    step_name  TEXT PRIMARY KEY CHECK ( step_name IN ('linear_pose_initialization',
                                                      'nonlinear_refinement',
                                                      'spline_interpolation',
                                                      'spline_nonlinear_refinement')),
    created_at DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP
);