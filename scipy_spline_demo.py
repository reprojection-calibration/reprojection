import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

x = np.array([5000, 5100, 5200, 5300])
y = np.array([[0, 0, 0], [1, 1, 1], [2, 2, 2], [3, 3, 3]])

cs = CubicSpline(x, y)

xs = np.arange(5000, 5200, 10)
f = cs(xs)

# plotting
ax = plt.figure().add_subplot(projection='3d')
ax.plot(f[:, 0], f[:, 1], f[:, 2], label='parametric curve')
ax.legend()

# plt.show()
