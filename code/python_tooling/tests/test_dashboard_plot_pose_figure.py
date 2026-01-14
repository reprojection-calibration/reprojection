import unittest

from dashboard.plot_pose_figure import plot_pose_figure


class TestDashboardPlotPoseFigure(unittest.TestCase):
    def test_plot_pose_figure(self):
        fig = plot_pose_figure([], [], "", "")
        self.assertEqual(fig, {})

        fig = plot_pose_figure([0], [[1, 2]], "", "")  # Pose data row is not 1X3 - error!
        self.assertEqual(fig, {})

        timestamps_ns = [20e9, 21e9, 25e9, 26e9]  # Must be sorted already!
        data = [[0, 0, 0], [1, 1, 1], [2, 2, 2], [3, 3, 3]]
        fig = plot_pose_figure(timestamps_ns, data, 'awesome_main_title', 'great_y_axis_title')

        # The three different scatters - one for each x, y, and z component
        self.assertEqual(len(fig['data']), 3)

        # For one scatter check that the data is set correctly and the x-axis calculated elapsed times are correct.
        self.assertEqual(fig['data'][0]['x'], (0.0, 1.0, 5.0, 6.0))
        self.assertEqual(fig['data'][0]['y'], (0, 1, 2, 3))

        # Figure properties we set from the outside
        self.assertEqual(fig['layout']['title']['text'], 'awesome_main_title')
        self.assertEqual(fig['layout']['yaxis']['title']['text'], 'great_y_axis_title')

        # Figure properties that are automatically set or calculated inside
        self.assertEqual(fig['layout']['xaxis']['tickmode'], 'array')
        self.assertEqual(fig['layout']['xaxis']['ticktext'], ('0s', '5s'))
        self.assertEqual(fig['layout']['xaxis']['tickvals'], (0.0, 5.0))
        self.assertEqual(fig['layout']['xaxis']['title']['text'], 'Time (s)')
