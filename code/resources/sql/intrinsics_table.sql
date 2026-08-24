CREATE TABLE IF NOT EXISTS intrinsics
(
    step_id      INTEGER NOT NULL,
    asset_id     INTEGER NOT NULL,
    camera_model TEXT    NOT NULL CHECK ( camera_model IN
                                          ('double_sphere', 'eucm', 'pinhole',
                                           'pinhole_radtan4', 'ucm')),
    data         TEXT    NOT NULL,

    FOREIGN KEY (step_id) REFERENCES steps (id) ON DELETE CASCADE,
    FOREIGN KEY (asset_id) REFERENCES assets (id) ON DELETE CASCADE,
    PRIMARY KEY (step_id, asset_id)
);