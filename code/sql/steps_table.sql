CREATE TABLE IF NOT EXISTS steps
(
    id           INTEGER PRIMARY KEY,
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

    UNIQUE (type, cache_key)
);