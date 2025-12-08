import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def load_points(file_path):
    """Load Nx3 points from a text file: each line 'x y z'."""
    return np.loadtxt(file_path)


def visualize_3d(measurement_file, control_file):
    # Load data
    measurements = load_points(measurement_file)
    controls = load_points(control_file)

    # Create 3D plot
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Scatter measurement inputs
    ax.scatter(
        measurements[:, 0], measurements[:, 1], measurements[:, 2],
        s=60, c='blue', label='3D Measurements'
    )

    # Scatter optimized control points
    ax.scatter(
        controls[:, 0], controls[:, 1], controls[:, 2],
        s=20, c='red', label='Optimized Control Points'
    )

    # Optional: connect control points to visualize spline structure
    ax.plot(
        controls[:, 0], controls[:, 1], controls[:, 2],
        c='red', linewidth=1.5, alpha=0.7
    )

    # Labels and legend
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("3D Measurements & Optimized Control Points")
    ax.legend()

    plt.show()


if __name__ == "__main__":
    # Change these to your filenames:
    visualize_3d("code/cmake-build-release-docker-reprojection/spline/measurements.txt",
                 "code/cmake-build-release-docker-reprojection/spline/control_points.txt")
