CREATE TABLE IF NOT EXISTS gravity
(
    step_id INTEGER PRIMARY KEY,
    gx      REAL NOT NULL,
    gy      REAL NOT NULL,
    gz      REAL NOT NULL,

    FOREIGN KEY (step_id) REFERENCES steps (id) ON DELETE CASCADE
);