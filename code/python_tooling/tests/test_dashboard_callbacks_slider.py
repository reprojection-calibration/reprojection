import unittest

from dashboard.callbacks.slider import (
    get_slider_properties,
    looping_increment,
    toggle_play,
)


class TestDashboardCallbacksSlider(unittest.TestCase):
    def test_update_slider_properties(self):
        marks, max_value = get_slider_properties("", {}, {})
        self.assertEqual(marks, {})
        self.assertEqual(max_value, 0)

        # Happy path success
        sensor = "/cam0/image_raw"
        n_frames = 20
        statistics = {sensor: {"total_frames": n_frames}}
        timestamps = {sensor: [int(i * 1e9) for i in range(n_frames)]}

        marks, max_value = get_slider_properties(sensor, statistics, timestamps)
        self.assertEqual(marks, {0: "0s", 5: "5s", 10: "10s", 15: "15s"})
        self.assertEqual(max_value, 19)

    def test_toggle_play(self):
        # The slider advances/plays by default which means the button needs to display the pause action.
        output = toggle_play(0)
        self.assertEqual(output, (False, "⏸ Pause"))

        output = toggle_play(1)
        self.assertEqual(output, (True, "▶ Play"))

        output = toggle_play(2)
        self.assertEqual(output, (False, "⏸ Pause"))

    def test_looping_increment(self):
        output = looping_increment(0, 10)
        self.assertEqual(output, 1)

        # Loop back to the start
        output = looping_increment(10, 10)
        self.assertEqual(output, 0)
