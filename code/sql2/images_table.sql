CREATE TABLE IF NOT EXISTS images
(
    artifact_id  INTEGER NOT NULL,
    asset_id     INTEGER NOT NULL,
    timestamp_ns INTEGER NOT NULL,
    image        BLOB,

    FOREIGN KEY (artifact_id) REFERENCES artifacts (id) ON DELETE CASCADE,
    FOREIGN KEY (asset_id) REFERENCES assets (id),
    PRIMARY KEY (artifact_id, asset_id, timestamp_ns)
);