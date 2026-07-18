INSERT INTO imu_errors (step_name, entity_id, imu_name, timestamp_ns, delta_omega_x, delta_omega_y, delta_omega_z,
                        delta_ax,
                        delta_ay, delta_az)
VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?);