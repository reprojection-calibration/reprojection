CREATE TABLE IF NOT EXISTS steps
(
    id          INTEGER PRIMARY KEY,
    run_id      INTEGER NOT NULL,
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
    cache_key  TEXT NOT NULL,
    created_at DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,

    FOREIGN KEY (run_id) REFERENCES runs (id) ON DELETE CASCADE
);