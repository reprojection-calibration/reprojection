CREATE TABLE IF NOT EXISTS spline_time_handler
(
    step_name   TEXT    NOT NULL,
    sensor_name TEXT    NOT NULL,
    t0_ns       INTEGER NOT NULL,
    delta_t_ns  INTEGER NOT NULL,

    FOREIGN KEY (step_name, sensor_name) REFERENCES calibration_steps (step_name, entity_id) ON DELETE CASCADE,
    PRIMARY KEY (step_name, sensor_name)
);