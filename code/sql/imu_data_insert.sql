INSERT INTO imu_data (timestamp_ns,
                      sensor_name,
                      omega_x,
                      omega_y,
                      omega_z,
                      ax,
                      ay,
                      az)
VALUES (?, ?, ?, ?, ?, ?, ?, ?);