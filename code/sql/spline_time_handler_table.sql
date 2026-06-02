CREATE TABLE IF NOT EXISTS spline_time_handler
(
    step_name   TEXT    NOT NULL,
    sensor_name TEXT    NOT NULL,
    t0_ns           INTEGER    NOT NULL,
    delta_t_ns           INTEGER    NOT NULL,
    PRIMARY KEY (step_name, sensor_name),
    FOREIGN KEY (step_name, sensor_name) REFERENCES calibration_steps ON DELETE CASCADE
);