import unittest

from dash.exceptions import PreventUpdate

from dashboard.callbacks.slider import advance_slider, toggle_play, update_slider_marks


class TestDashboardCallbacksSlider(unittest.TestCase):
    def test_update_slider_marks(self):
        self.assertRaises(PreventUpdate, update_slider_marks, None, None)

        output = update_slider_marks("", [None, {}])
        self.assertEqual(output, {})

        # Happy path success
        selected_sensor = "/cam0/image_raw"
        timestamps_sorted = {selected_sensor: [int(i * 1e9) for i in range(20)]}

        output = update_slider_marks(selected_sensor, [None, timestamps_sorted])
        self.assertEqual(output, {0: "0s", 5: "5s", 10: "10s", 15: "15s"})

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
