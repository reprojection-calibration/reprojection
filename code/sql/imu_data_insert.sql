INSERT INTO imu_data (sensor_name,
                      timestamp_ns,
                      omega_x,
                      omega_y,
                      omega_z,
                      ax,
                      ay,
                      az)
VALUES (?, ?, ?, ?, ?, ?, ?, ?);