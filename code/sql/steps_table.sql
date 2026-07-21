CREATE TABLE IF NOT EXISTS steps
(
    id           INTEGER PRIMARY KEY,
    recording_id INTEGER,
    run_id       INTEGER,
    type         TEXT     NOT NULL CHECK ( type IN ('bundle_adjustment',
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
    cache_key    TEXT,
    created_at   DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,

    CHECK ((recording_id IS NOT NULL AND run_id IS NULL) OR
           (recording_id IS NULL AND run_id IS NOT NULL)),
    FOREIGN KEY (recording_id) REFERENCES recordings (id) ON DELETE CASCADE,
    FOREIGN KEY (run_id) REFERENCES runs (id) ON DELETE CASCADE,
    UNIQUE (run_id, type)
);