import matplotlib.pyplot as plt
import numpy as np

import pytransform3d.camera as pc
import pytransform3d.transformations as pt

filename = "code/cmake-build-release-docker-reprojection/demos/nl_poses.txt"
poses = np.loadtxt(filename)

# default parameters of a camera in Blender
sensor_size = np.array([0.01, 0.01])
intrinsic_matrix = np.array(
    [
        [0.05, 0, sensor_size[0] / 2.0],
        [0, 0.05, sensor_size[1] / 2.0],
        [0, 0, 1],
    ]
)
virtual_image_distance = 0.1

plt.ion()

for se3_i in poses:
    cam2world = pt.transform_from_exponential_coordinates(se3_i)
    pt.plot_transform(A2B=cam2world, s=0.1)
    plt.pause(0.0001)  # Allow the figure to update

plt.ioff()
plt.show()
