import unittest

from dash.exceptions import PreventUpdate

from dashboard.callbacks.statistics import update_statistics


class TestDashboardCallbacksStatistics(unittest.TestCase):
    def test_update_statistics(self):
        self.assertRaises(PreventUpdate, update_statistics, None, None)

        sensor = "/cam0/image_raw"
        statistics = {sensor: {'statistic_1': 0, 'statistic_2': 100, 'statistic_3': 101, 'statistic_4': 102}}

        output = update_statistics(sensor, [statistics, None])

        # Four statistic rows to display
        self.assertEqual(len(output), 4)

        # Check the first statistic which should display a red box because there are no frames
        div_statistic_1 = output[0]
        self.assertEqual(len(div_statistic_1.children), 3)

        div_colored_box_1 = div_statistic_1.children[0]
        self.assertEqual(div_colored_box_1.style['backgroundColor'], 'red')

        div_counter_1 = div_statistic_1.children[1]
        self.assertEqual(div_counter_1.children,0)

        div_label_1 = div_statistic_1.children[2]
        self.assertEqual(div_label_1.children,'statistic_1')
