CREATE TABLE IF NOT EXISTS calibration_steps
(
    step_name  TEXT     NOT NULL CHECK ( step_name IN ('bundle_adjustment',
                                                       'camera_info',
                                                       'extrinsic_initialization',
                                                       'feature_extraction',
                                                       'image_loading',
                                                       'intrinsic_initialization',
                                                       'pose_initialization',
                                                       'spline_initialization',
                                                       'target_info')),
    entity_id  TEXT     NOT NULL,
    cache_key  TEXT,
    created_at DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,

    FOREIGN KEY (entity_id) REFERENCES entity ON DELETE CASCADE,
    PRIMARY KEY (step_name, entity_id)
);