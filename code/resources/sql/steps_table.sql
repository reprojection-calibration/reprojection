CREATE TABLE IF NOT EXISTS steps
(
    id         INTEGER PRIMARY KEY,
    type       TEXT     NOT NULL CHECK ( type IN ('bundle_adjustment',
                                                  'camera_info',
                                                  'feature_extraction',
                                                  'image_loading',
                                                  'imu_data_loading',
                                                  'intrinsic_init',
                                                  'pose_init',
                                                  'spline_init',
                                                  'stereo_rig_init',
                                                  'stereo_rig_opt',
                                                  'target_info',
                                                  'visual_inertial_init',
                                                  'visual_inertial_opt')),
    cache_key  TEXT,
    created_at DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,

    UNIQUE (id, type),
    UNIQUE (type, cache_key)
);