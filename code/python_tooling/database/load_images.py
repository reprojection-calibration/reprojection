# NOTE(Jack): The images hold a special place in the database that sets it apart from all tables. And that is, it is the
# only table with NO foreign key relationship. That means that it defines what can and what can not exist as data in the
# rest of the database. All other tables have one way or another a foreign key relationship on the image table.
def image_df_to_camera_calibration_data(df):
    data = {}
    for index, row in df.iterrows():
        sensor = row["sensor_name"]
        timestamp = int(row["timestamp_ns"])

        if sensor not in data:
            data[sensor] = {"frames": {}}

        data[sensor]["frames"][timestamp] = {"image": row["data"]}

    return data
