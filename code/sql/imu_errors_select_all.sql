SELECT step_name,
       sensor_name,
       timestamp_ns,
       delta_omega_x,
       delta_omega_y,
       delta_omega_z,
       delta_ax,
       delta_ay,
       delta_az
FROM imu_errors
ORDER BY timestamp_ns;