CREATE TABLE IF NOT EXISTS runs
(
    id            INTEGER PRIMARY KEY,
    recording_id  INTEGER  NOT NULL,
    configuration TEXT     NOT NULL,
    created_at    DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,

    FOREIGN KEY (recording_id) REFERENCES recordings (id) ON DELETE CASCADE
);