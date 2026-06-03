SELECT timestamp_ns, delta_omega_x, delta_omega_y, delta_omega_z, delta_ax, delta_ay, delta_az
FROM imu_error
WHERE step_name = ?
  AND sensor_name = ?
ORDER BY timestamp_ns;