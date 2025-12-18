SELECT timestamp_ns, omega_x, omega_y, omega_z, ax, ay, az
FROM imu_data
WHERE sensor_name = ?
ORDER BY timestamp_ns ASC;