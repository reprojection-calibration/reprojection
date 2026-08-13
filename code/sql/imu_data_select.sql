SELECT timestamp_ns, omega_x, omega_y, omega_z, ax, ay, az
FROM imu_data
WHERE step_id = ?
  AND asset_id = ?;