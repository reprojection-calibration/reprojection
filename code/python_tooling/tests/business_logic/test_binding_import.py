import unittest

from projection_function_python_binding import ImageBounds


class TestBindingImport(unittest.TestCase):
    def test_image_bounds(self):
        bounds = ImageBounds(1, 2, 3, 4)
        self.assertEqual(bounds.u_min, 1)
        self.assertEqual(bounds.u_max, 2)
        self.assertEqual(bounds.v_min, 3)
        self.assertEqual(bounds.v_max, 4)


if __name__ == "__main__":
    unittest.main()
