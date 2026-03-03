import numpy as np

from generated.extracted_target_pb2 import ArrayX2dProto, ExtractedTargetProto


def build_pixels(msg_data):
    rows = msg_data.pixel_rows
    data = np.array(msg_data.pixel_data, dtype=np.float64)

    if rows == 0 or data.size == 0:
        return np.empty((0, 2))
    else:
        # NOTE(Jack): We need the transpose here because eigen stores the data column wise. At least I think that is
        # why :)
        return data.reshape(2, rows).transpose()


def build_points(msg_data):
    rows = msg_data.point_rows
    data = np.array(msg_data.point_data, dtype=np.float64)

    if rows == 0 or data.size == 0:
        return np.empty((0, 3))
    else:
        return data.reshape(3, rows).transpose()


def build_indices(msg_data):
    rows = msg_data.indices_rows
    data = np.array(msg_data.indices_data, dtype=np.int32)

    if rows == 0 or data.size == 0:
        return np.empty((0, 2))
    else:
        return data.reshape(2, rows).transpose()


# TODO(Jack): I really am not sure here, but is storing a dict in a pandas datatable a crime? Might be but for not the
# clarity of the abstraction takes precedence! But lets be ready to refactor this if it turns into a problem.
def parse_extracted_target_proto(blob):
    msg = ExtractedTargetProto()
    msg.ParseFromString(blob)

    return {
        "pixels": build_pixels(msg.bundle).tolist(),
        "points": build_points(msg.bundle).tolist(),
        "indices": build_indices(msg).tolist(),
    }


# TODO(Jack): Due to undesirable dual definition of pixels with the extracted target and the array which stores the
#  reprojection error we need essentially duplicated this function in two places. One day when we unify the
#  representation we can combine them and use one common function.
def build_array_x2d(msg_data):
    rows = msg_data.rows
    data = np.array(msg_data.array_data, dtype=np.float64)

    if rows == 0 or data.size == 0:
        return np.empty((0, 2))
    else:
        return data.reshape(2, rows).transpose()


def parse_array_x2d_proto(blob):
    msg = ArrayX2dProto()
    msg.ParseFromString(blob)

    return build_array_x2d(msg).tolist()
