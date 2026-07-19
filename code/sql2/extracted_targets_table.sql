CREATE TABLE IF NOT EXISTS extracted_targets
(
    artifact_id        INTEGER NOT NULL,
    source_artifact_id INTEGER NOT NULL,
    asset_id           INTEGER NOT NULL,
    timestamp_ns       INTEGER NOT NULL,
    data               BLOB,

    FOREIGN KEY (artifact_id) REFERENCES artifacts (id) ON DELETE CASCADE,
    FOREIGN KEY (source_artifact_id, asset_id, timestamp_ns) REFERENCES images (artifact_id, asset_id, timestamp_ns) ON DELETE CASCADE,
    PRIMARY KEY (artifact_id, source_artifact_id, asset_id, timestamp_ns)
);