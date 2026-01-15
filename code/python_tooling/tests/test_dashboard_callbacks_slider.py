import unittest

from dashboard.callbacks.slider import (
    advance_slider,
    toggle_play,
    update_slider_properties,
)


class TestDashboardCallbacksSlider(unittest.TestCase):
    def test_update_slider_properties(self):
        marks, max_value = update_slider_properties(None, None)
        self.assertEqual(marks, {})
        self.assertEqual(max_value, 0)

        marks, max_value = update_slider_properties("", [{}, {}])
        self.assertEqual(marks, {})
        self.assertEqual(max_value, 0)

        # Happy path success
        sensor = "/cam0/image_raw"
        n_frames = 20
        statistics = {sensor: {"total_frames": n_frames}}
        timestamps_sorted = {sensor: [int(i * 1e9) for i in range(n_frames)]}

        marks, max_value = update_slider_properties(
            sensor, [statistics, timestamps_sorted]
        )
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

    def test_advance_slider(self):
        # Note that the Input("play-interval", "n_intervals") is only here to trigger this function! We do not use the
        # provided n_intervals (the first argument to this function) in any real way and instead use the
        # State("frame-id-slider", "value") to track our frame idx.
        output = advance_slider(None, None, None)
        self.assertEqual(output, 0)

        output = advance_slider(None, 0, 10)
        self.assertEqual(output, 1)

        # Loop back to the start
        output = advance_slider(None, 10, 10)
        self.assertEqual(output, 0)
