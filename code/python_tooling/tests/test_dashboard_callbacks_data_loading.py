import unittest

from dashboard.callbacks.data_loading import (
    load_database_to_store,
    refresh_database_list,
    refresh_sensor_list,
)


class TestDashboardCallbacksSlider(unittest.TestCase):
    def test_refresh_database_list(self):
        list, default_value = refresh_database_list(
            0
        )  # The input n_clicks is only use as a trigger
        self.assertEqual(list, [])
        self.assertEqual(default_value, "")
