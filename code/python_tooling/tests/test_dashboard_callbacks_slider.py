import unittest

from reference_data import FullData as FD
from reference_data import InvalidData as ID
from reference_data import SkeletonData as SD

from dashboard.callbacks.slider import (
    toggle_play_callback,
)
from dashboard.tools.slider import (
    looping_increment,
    update_slider_properties,
)
from database.types import SensorType


class DashboardSlider(unittest.TestCase):
    def test_update_slider_properties_invalid_data(self):
        marks, max_value = update_slider_properties("", ID.metadata, SensorType.Camera)
        self.assertEqual(marks, {})
        self.assertEqual(max_value, 0)

    def test_update_slider_properties_skeleton_data(self):
        marks, max_value = update_slider_properties("", SD.metadata, SensorType.Camera)
        self.assertEqual(marks, {})
        self.assertEqual(max_value, 0)

    def test_update_slider_properties(self):
        marks, max_value = update_slider_properties(
            "/cam0/image_raw", FD.metadata, SensorType.Camera
        )
        self.assertEqual(marks, {0: "0s"})
        self.assertEqual(max_value, 0)

    def test_toggle_play_callback(self):
        # The slider advances/plays by default which means the button needs to display the pause action.
        output = toggle_play_callback(0)
        self.assertEqual(output, (False, "⏸ Pause"))

        output = toggle_play_callback(1)
        self.assertEqual(output, (True, "▶ Play"))

        output = toggle_play_callback(2)
        self.assertEqual(output, (False, "⏸ Pause"))

    def test_looping_increment(self):
        output = looping_increment(0, 10)
        self.assertEqual(output, 1)

        # Loop back to the start
        output = looping_increment(10, 10)
        self.assertEqual(output, 0)
