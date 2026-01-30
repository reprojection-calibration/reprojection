SELECT timestamp_ns,
       sensor_name,
       omega_x,
       omega_y,
       omega_z,
       ax,
       ay,
       az
FROM imu_data
ORDER BY timestamp_ns ASC;