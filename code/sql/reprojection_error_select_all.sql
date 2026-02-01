SELECT timestamp_ns,
       sensor_name,
       data,
       type
FROM reprojection_error
ORDER BY timestamp_ns ASC;