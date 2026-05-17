from build_camera_tomls import build_camera_tomls

from database.sql_table_loading import (
    load_camera_info_table,
    load_camera_intrinsics_table,
)


def main():
    db_path = "../test_data/dataset-calib-imu4_512_16.db3"

    camera_info = load_camera_info_table(db_path)
    camera_intrinsics = load_camera_intrinsics_table(db_path)

    result = build_camera_tomls(camera_info, camera_intrinsics)

    with open("cameras.toml", "w") as f:
        f.write(result)


if __name__ == "__main__":
    main()
