import unittest

from dashboard.tools.plot_r3_timeseries import (
    R3TimeseriesFigureConfig,
    build_r3_timeseries_figure,
    timeseries_plot,
)

# def build_r3_timeseries_figure(
#        timestamps_ns,
#        data,
#        config: R3TimeseriesFigureConfig,
#        fig=None,
# ):


class TestDashboardToolsPlotR3Timeseries(unittest.TestCase):
    def test_build_r3_timeseries_figure(self):
        # Mismatching timestamp and data lengths
        fig = build_r3_timeseries_figure([1], [1, 1], None)
        self.assertEqual(fig, {})

        # No timestamps at all
        fig = build_r3_timeseries_figure([], [], None)
        self.assertEqual(fig, {})

        # Data row is not 1X3 - error!
        fig = build_r3_timeseries_figure([0], [[1, 2]], None)
        self.assertEqual(fig, {})

        timestamps_ns = [20e9, 21e9, 25e9, 26e9]  # Must be sorted already!
        data = [[0, 0, 0], [1, 1, 1], [2, 2, 2], [3, 3, 3]]
        config = R3TimeseriesFigureConfig("awesome_main_title", "great_y_axis_title")
        fig = build_r3_timeseries_figure(
            timestamps_ns,
            data,
            config,
        )

        # The three different scatters - one for each x, y, and z component
        self.assertEqual(len(fig["data"]), 3)

        # For one scatter check that the data is set correctly and the x-axis calculated elapsed times are correct.
        self.assertEqual(fig["data"][0]["x"], (0.0, 1.0, 5.0, 6.0))
        self.assertEqual(fig["data"][0]["y"], (0, 1, 2, 3))

        # Figure properties we set from the outside
        self.assertEqual(fig["layout"]["title"]["text"], "awesome_main_title")
        self.assertEqual(fig["layout"]["yaxis"]["title"]["text"], "great_y_axis_title")

    def test_timeseries_plot(self):
        timestamps_ns = [20e9, 21e9, 25e9, 26e9]  # Must be sorted already!
        fig = timeseries_plot(timestamps_ns, 5)

        self.assertEqual(fig["layout"]["xaxis"]["tickmode"], "array")
        self.assertEqual(fig["layout"]["xaxis"]["ticktext"], ("0s", "5s"))
        self.assertEqual(fig["layout"]["xaxis"]["tickvals"], (0.0, 5.0))
        self.assertEqual(fig["layout"]["xaxis"]["title"]["text"], "Time (s)")
