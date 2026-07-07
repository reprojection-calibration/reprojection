import numpy as np
from scipy.spatial.transform import RigidTransform as TF
from scipy.spatial.transform import Rotation as R


# TODO(Jack): Is there a better name for this function?
# Little se3 is a 6 vector [rx,ry,rz,x,y,z] and big SE3 is a 4x4 homogeneous transform [R,t].
def Se3ToMat(se3):
    # NOTE(Jack): Make a copy otherwise scipy can complain about trying to work with immutable memory
    se3 = np.asarray(se3, dtype=np.float64).copy().reshape(-1)
    if se3.shape != (6,):
        raise ValueError(f"Expected se3 vector with shape (6,), got {se3.shape}")

    aa = R.from_rotvec(se3[:3]).as_matrix()
    t = se3[3:]

    SE3 = np.eye(4)
    SE3[:3, :3] = aa
    SE3[:3, 3] = t

    return SE3


def InvertSe3(se3):
    SE3 = Se3ToMat(se3)

    # Invert it
    tf = TF.from_matrix(SE3)
    tf_inv = tf.inv()

    # Convert back to se3 and return it
    aa = tf_inv.rotation.as_rotvec()
    t = tf_inv.translation

    return list(np.concatenate([aa, t]))
