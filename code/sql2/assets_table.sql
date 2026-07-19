CREATE TABLE IF NOT EXISTS assets
(
    id      INTEGER PRIMARY KEY,
    type    TEXT    NOT NULL CHECK (
        type IN ('camera', 'imu', 'target')
        ),
    "index" INTEGER NOT NULL CHECK ("index" >= 0),
    name    TEXT    NOT NULL UNIQUE,

    UNIQUE (type, "index")
);