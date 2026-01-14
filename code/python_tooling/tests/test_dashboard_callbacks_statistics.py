import unittest

from dash.exceptions import PreventUpdate

from dashboard.callbacks.statistics import update_statistics


class TestDashboardCallbacksStatistics(unittest.TestCase):
    def test_update_statistics(self):
        self.assertRaises(PreventUpdate, update_statistics, None, None)

        sensor = "/cam0/image_raw"
        statistics = {sensor: {'statistic_1': 0, 'statistic_2': 100, 'statistic_3': 101, 'statistic_4': 102}}
        output = update_statistics(sensor, [statistics, None])

        # Four statistics to display
        self.assertEqual(len(output), 4)

        def assert_statistic_row_element(div, color, value, label):
            div_colored_box = div.children[0]
            self.assertEqual(div_colored_box.style['backgroundColor'], color)

            div_counter = div.children[1]
            self.assertEqual(div_counter.children, value)

            div_label = div.children[2]
            self.assertEqual(div_label.children, label)

        # Check the first statistic which should display a red box because there are no frames for statistic_1
        div_statistic_1 = output[0]
        self.assertEqual(len(div_statistic_1.children), 3)
        assert_statistic_row_element(div_statistic_1, 'red', 0, 'statistic_1')

        # Check any of the other three statistics to make sure it is a green box
        div_statistic_2 = output[1]
        self.assertEqual(len(div_statistic_2.children), 3)
        assert_statistic_row_element(div_statistic_2, 'green', 100, 'statistic_2')
