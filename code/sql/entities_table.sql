CREATE TABLE IF NOT EXISTS entities
(
    entity_id   TEXT     NOT NULL,
    entity_type TEXT     NOT NULL CHECK ( entity_type IN ('camera',
                                                          'extrinsic',
                                                          'imu',
                                                          'target')),
    created_at  DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    PRIMARY KEY (entity_id)
);