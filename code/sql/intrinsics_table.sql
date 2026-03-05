CREATE TABLE intrinsics
(
    step_name   TEXT    NOT NULL,
    sensor_name TEXT    NOT NULL,
    model       TEXT    NOT NULL CHECK ( model IN ('double_sphere',
                                                   'pinhole',
                                                   'pinhole_radtan4',
                                                   'unified_camera_model')),
    width       INTEGER NOT NULL,
    height      INTEGER NOT NULL,
    intrinsics  TEXT    NOT NULL,
    PRIMARY KEY (step_name, sensor_name),
    FOREIGN KEY (step_name) REFERENCES calibration_steps (step_name)
);