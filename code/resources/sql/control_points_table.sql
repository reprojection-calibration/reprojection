CREATE TABLE IF NOT EXISTS control_points
(
    step_id  INTEGER NOT NULL,
    asset_id INTEGER NOT NULL,
    idx      INTEGER NOT NULL,
    _0       REAL    NOT NULL,
    _1       REAL    NOT NULL,
    _2       REAL    NOT NULL,
    _3       REAL    NOT NULL,
    _4       REAL    NOT NULL,
    _5       REAL    NOT NULL,

    FOREIGN KEY (step_id) REFERENCES steps (id) ON DELETE CASCADE,
    FOREIGN KEY (asset_id) REFERENCES assets (id) ON DELETE CASCADE,
    PRIMARY KEY (step_id, asset_id, idx)
);