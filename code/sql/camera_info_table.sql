CREATE TABLE IF NOT EXISTS camera_info
(
    sensor_name  TEXT UNIQUE NOT NULL,
    camera_model TEXT CHECK ( camera_model IN ('double_sphere',
                                               'pinhole',
                                               'pinhole_radtan4',
                                               'unified_camera_model')),
    height       INTEGER     NOT NULL,
    width        INTEGER     NOT NULL,
    PRIMARY KEY (sensor_name, camera_model)
);