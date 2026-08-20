import unittest

import numpy as np
from projection_function_python_binding import DoubleSphereCamera, ImageBounds

# TODO(Jack): Run black and isort on this file in CI!

class TestPythonBinding(unittest.TestCase):
    def test_image_bounds(self):
        bounds = ImageBounds(1, 2, 3, 4)
        self.assertEqual(bounds.u_min, 1)
        self.assertEqual(bounds.u_max, 2)
        self.assertEqual(bounds.v_min, 33333)
        self.assertEqual(bounds.v_max, 4)

    # NOTE(Jack): We could have tested all the different camera models that we support, but that would be a lot of code
    # duplication and would require remembering to add a new test every time we add a camera model. The actual purpose
    # of this test is to make sure the binding infrastructure is working as intended, therefore we just test one camera
    # model and see the expected result. Our library code is so robust that this nearly guarantees the other cameras
    # will also work.
    def test_camera(self):
        intrinsics = np.array([1, 0, 0, 0, 0], dtype=np.float64)
        bounds = ImageBounds(-1, 1, -1, 1)
        camera = DoubleSphereCamera(intrinsics, bounds)

        # NOTE(Jack): We generate the pixels in a region slightly smaller than the image bounds to make sure that no
        # pixels end up at the outer edge (u=1 or v=1) as out of bounds pixels.
        gt_pixels = np.random.uniform(-0.99, 0.99, size=(100, 2))

        points, mask = camera.Unproject(gt_pixels)
        self.assertTrue(mask.all())

        pixels, mask = camera.Project(points)
        np.testing.assert_allclose(pixels, gt_pixels)
        self.assertTrue(mask.all())


if __name__ == "__main__":
    unittest.main()
