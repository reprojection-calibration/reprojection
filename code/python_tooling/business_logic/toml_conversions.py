import tomllib

from database.types import CameraModel


def toml_to_intrinsic_array(intrinsics_str, camera_model):
    intrinsics_toml = tomllib.loads(intrinsics_str)

    # NOTE(Jack): All camera models so far are a derivative of teh pinhole camera model therefore we build the basic
    # pinhole intrinsics here and add to them as needed depending on the camera model.
    intrinsics = [intrinsics_toml["f"], intrinsics_toml["cx"], intrinsics_toml["cy"]]

    if camera_model == CameraModel.DoubleSphere:
        ds = [intrinsics_toml["xi"], intrinsics_toml["alpha"]]
        intrinsics.extend(ds)
    elif camera_model == CameraModel.Pinhole:
        pass
    elif camera_model == CameraModel.PinholeRadtan4:
        radtan4 = [
            intrinsics_toml["k1"],
            intrinsics_toml["k2"],
            intrinsics_toml["p1"],
            intrinsics_toml["p2"],
        ]
        intrinsics.extend(radtan4)
    elif camera_model == CameraModel.UnifiedCameraModel:
        ucm = [intrinsics_toml["xi"]]
        intrinsics.extend(ucm)
    else:
        raise ValueError(
            "IMPLEMENTATION ERROR - toml_to_intrinsic_array() - unimplemented camera model requested!"
        )

    return intrinsics
