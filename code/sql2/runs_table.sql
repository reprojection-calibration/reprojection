CREATE TABLE IF NOT EXISTS runs
(
    id           INTEGER PRIMARY KEY,
    recording_id INTEGER  NOT NULL,
    config_hash  TEXT     NOT NULL,
    config       TEXT     NOT NULL,
    created_at   DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,

    FOREIGN KEY (recording_id) REFERENCES recordings (id) ON DELETE CASCADE,
    UNIQUE (recording_id, config_hash)
);