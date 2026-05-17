from database.sql_table_loading import (
    load_camera_info_table,
    load_camera_intrinsics_table,
)

from database.types import CameraModel
import tomllib


# TEST
# TEST
# TEST
def toml_to_intrinsic_array(intrinsics_str, camera_model):
    intrinsics_toml = tomllib.loads(intrinsics_str)

    # NOTE(Jack): All camera models so far are a derivative of teh pinhole camera model therefore we build the basic
    # pinhole intrinsics here and add to them as needed depending on the camera model.
    intrinsics = [intrinsics_toml['f'], intrinsics_toml['cx'], intrinsics_toml['cy']]

    if camera_model == CameraModel.DoubleSphere:
        ds = [intrinsics_toml['xi'], intrinsics_toml['alpha']]
        intrinsics.extend(ds)
    elif camera_model == CameraModel.Pinhole:
        pass
    elif camera_model == CameraModel.PinholeRadtan4:
        radtan4 = [intrinsics_toml['k1'], intrinsics_toml['k2'], intrinsics_toml['p1'], intrinsics_toml['p2']]
        intrinsics.extend(radtan4)
    elif camera_model == CameraModel.UnifiedCameraModel:
        ucm = [intrinsics_toml['xi']]
        intrinsics.extend(ucm)
    else:
        raise ValueError("IMPLEMENTATION ERROR - toml_to_intrinsic_array() - unimplemented camera model requested!")

    return intrinsics


def write_sensor_tomls(camera_info, camera_intrinsics):
    refined_intrinsics = camera_intrinsics[
        camera_intrinsics["step_name"] == "camera_nonlinear_refinement"
        ]

    for _, sensor in camera_info.iterrows():
        sensor_name = sensor["sensor_name"]

        matches = refined_intrinsics[refined_intrinsics["sensor_name"] == sensor_name]

        if matches.empty:
            continue

        intrinsics = matches.iloc[0]["intrinsics"]
        camera_model = CameraModel(sensor["camera_model"])
        print(toml_to_intrinsic_array(intrinsics, camera_model))


def main():
    db_path = "../test_data/dataset-calib-imu4_512_16.db3"

    camera_info = load_camera_info_table(db_path)
    camera_intrinsics = load_camera_intrinsics_table(db_path)

    write_sensor_tomls(camera_info, camera_intrinsics)


if __name__ == "__main__":
    main()
