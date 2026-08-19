CREATE TABLE IF NOT EXISTS spline_info
(
    step_id     INTEGER NOT NULL,
    asset_id    INTEGER NOT NULL,
    spline_type TEXT    NOT NULL CHECK ( spline_type IN ('pose')),
    t0_ns       INTEGER NOT NULL,
    delta_t_ns  INTEGER NOT NULL,

    FOREIGN KEY (step_id) REFERENCES steps (id) ON DELETE CASCADE,
    FOREIGN KEY (asset_id) REFERENCES assets (id) ON DELETE CASCADE,
    PRIMARY KEY (step_id, asset_id)
);