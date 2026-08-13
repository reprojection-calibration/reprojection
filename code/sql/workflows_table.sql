CREATE TABLE IF NOT EXISTS workflows
(
    id   INTEGER PRIMARY KEY,
    type TEXT NOT NULL CHECK ( type IN ('cam', 'cam_imu')),
    signature TEXT NOT NULL UNIQUE
);