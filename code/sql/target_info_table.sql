CREATE TABLE IF NOT EXISTS target_info
(
    step_id        INTEGER NOT NULL,
    asset_id       INTEGER NOT NULL,
    target_type    TEXT        NOT NULL CHECK ( target_type IN ('aprilgrid3', 'checkerboard', 'circle_grid')),
    height         INTEGER     NOT NULL,
    width          INTEGER     NOT NULL,
    unit_dimension REAL        NOT NULL,
    asymmetric     INTEGER     NOT NULL CHECK ( asymmetric IN (0, 1)),

    FOREIGN KEY (step_id) REFERENCES steps (id) ON DELETE CASCADE,
    FOREIGN KEY (asset_id) REFERENCES assets (id) ON DELETE CASCADE,
    PRIMARY KEY (step_id, asset_id)
);