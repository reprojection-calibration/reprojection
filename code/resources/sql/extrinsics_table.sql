CREATE TABLE IF NOT EXISTS extrinsics
(
    step_id    INTEGER NOT NULL,
    asset_a_id INTEGER NOT NULL,
    asset_b_id INTEGER NOT NULL,
    rx         REAL    NOT NULL,
    ry         REAL    NOT NULL,
    rz         REAL    NOT NULL,
    x          REAL    NOT NULL,
    y          REAL    NOT NULL,
    z          REAL    NOT NULL,

    FOREIGN KEY (step_id) REFERENCES steps (id) ON DELETE CASCADE,
    FOREIGN KEY (asset_a_id) REFERENCES assets (id) ON DELETE CASCADE,
    FOREIGN KEY (asset_b_id) REFERENCES assets (id) ON DELETE CASCADE,
    PRIMARY KEY (step_id, asset_a_id, asset_b_id)
);