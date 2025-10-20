import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def read_transforms_from_csv(filename):
    """
    Reads the transformation matrices (flattened into one row) from a CSV file
    and returns them as a list of 4x4 numpy arrays.
    """
    transforms = []
    with open(filename, 'r') as file:
        lines = file.readlines()

        for line in lines:
            # Split the row by commas, convert each element to float, and reshape to 4x4
            matrix_values = [float(x) for x in line.strip().split(',')]
            if len(matrix_values) == 16:
                matrix = np.array(matrix_values).reshape(4, 4)
                transforms.append(matrix)
            else:
                print(f"Warning: Invalid matrix with {len(matrix_values)} elements.")

    return transforms

def plot_transforms(transforms):
    """
    Plots the translation part of each transformation in a 3D plot.
    """
    # Set up the plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Iterate through each transformation matrix
    for transform in transforms:
        # Extract the translation vector (last column of the matrix)
        translation = transform[:3, 3]

        # Plot the translation as a point
        ax.scatter(translation[0], translation[1], translation[2], color='b', marker='o')

        # Optionally, visualize the coordinate axes at each transformation
        origin = translation
        x_axis = translation + transform[:3, 0]  # First column (X direction)
        y_axis = translation + transform[:3, 1]  # Second column (Y direction)
        z_axis = translation + transform[:3, 2]  # Third column (Z direction)

        ax.quiver(*origin, *(x_axis - origin), color='r', length=0.5)
        ax.quiver(*origin, *(y_axis - origin), color='g', length=0.5)
        ax.quiver(*origin, *(z_axis - origin), color='b', length=0.5)

    # Set plot limits
    ax.set_xlim([-10, 10])
    ax.set_ylim([-10, 10])
    ax.set_zlim([-10, 10])

    # Labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Poses from Transformation Matrices')

    # Show the plot
    plt.show()

if __name__ == "__main__":
    # Read the transformations from a CSV file
    transforms = read_transforms_from_csv("code/cmake-build-debug-docker-reprojection/testing_mocks/sphere_cameras.txt")

    # Plot the transformations
    plot_transforms(transforms)