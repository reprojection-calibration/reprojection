CREATE TABLE IF NOT EXISTS workflow_steps
(
    workflow_id INTEGER NOT NULL,
    type        TEXT    NOT NULL CHECK ( type IN ('bundle_adjustment',
                                                  'camera_info',
                                                  'extrinsic_initialization',
                                                  'extrinsic_optimization',
                                                  'feature_extraction',
                                                  'image_loading',
                                                  'imu_data_loading',
                                                  'intrinsic_initialization',
                                                  'pose_initialization',
                                                  'spline_initialization',
                                                  'target_info')),
    step_id     INTEGER NOT NULL,

    FOREIGN KEY (workflow_id) REFERENCES workflows (id) ON DELETE CASCADE,
    FOREIGN KEY (step_id) REFERENCES steps (id),
    PRIMARY KEY (workflow_id, type)
);