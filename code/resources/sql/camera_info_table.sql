CREATE TABLE IF NOT EXISTS camera_info
(
    step_id      INTEGER NOT NULL,
    asset_id     INTEGER NOT NULL,
    camera_model TEXT    NOT NULL CHECK ( camera_model IN
                                          ('double_sphere', 'extended_unified_camera_model', 'pinhole',
                                           'pinhole_radtan4', 'unified_camera_model')),
    height       INTEGER NOT NULL,
    width        INTEGER NOT NULL,

    FOREIGN KEY (step_id) REFERENCES steps (id) ON DELETE CASCADE,
    FOREIGN KEY (asset_id) REFERENCES assets (id) ON DELETE CASCADE,
    PRIMARY KEY (step_id, asset_id)
);