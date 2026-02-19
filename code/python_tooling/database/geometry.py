import numpy as np
from scipy.spatial.transform import RigidTransform as TF
from scipy.spatial.transform import Rotation as R

# TODO(Jack): One day it might make sense to use generic geometry methods in both the dashboard and database, but for
#  now we just need it for the camera pose loading so we leave it here.


# Little se3 is a 6 vector [rx,ry,rz,x,y,z] and big SE3 is a 4x4 homogeneous transform [R,t]
def InvertSe3(se3):
    # Build the transformation matrix
    aa = R.from_rotvec(se3[:3]).as_matrix()
    t = se3[3:]

    SE3 = np.eye(4)
    SE3[:3, :3] = aa
    SE3[:3, 3] = t

    # Invert it
    tf = TF.from_matrix(SE3)

    # Convert back to se3 and return it
    tf_inv = tf.inv()
    aa = tf_inv.rotation.as_rotvec()
    t = tf_inv.translation

    return list(np.concatenate([aa, t]))
