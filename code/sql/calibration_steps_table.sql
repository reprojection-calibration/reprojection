CREATE TABLE IF NOT EXISTS calibration_steps
(
    step_name  TEXT PRIMARY KEY CHECK ( step_name IN ('focal_length_initialization',
                                                      'linear_pose_initialization',
                                                      'nonlinear_refinement',
                                                      'spline_initialization',
                                                      'spline_nonlinear_refinement')),
    cache_key  TEXT, -- TODO(jack): Can we make this have a not null constraint?
    created_at DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP
);