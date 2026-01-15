import unittest

from dashboard.callbacks.pose_graph import init_pose_graph_figures
from database.types import PoseType
import plotly.graph_objects as go


class TestDashboardCallbacksPoseGraph(unittest.TestCase):
    def test_init_pose_graph_figures(self):
        rot_fig, trans_fig = init_pose_graph_figures(None, None, None, None)
        self.assertEqual(rot_fig, {})
        self.assertEqual(trans_fig, {})

        sensor = "/cam0/image_raw"

        # Requested sensor not found in indexable_timestamps.
        rot_fig, trans_fig = init_pose_graph_figures(
            sensor, PoseType.Initial, {}, [None, {"/not/cam0/image_raw": None}]
        )
        self.assertEqual(rot_fig, {})
        self.assertEqual(trans_fig, {})

        # Sensor found in processed_data not found in raw_data.
        self.assertRaises(
            RuntimeError,
            init_pose_graph_figures,
            sensor,
            PoseType.Initial,
            {"/not/cam0/image_raw": None},
            [None, {sensor: []}],
        )

        # raw_data does not contain the 'frames' key.
        self.assertRaises(
            RuntimeError,
            init_pose_graph_figures,
            sensor,
            PoseType.Initial,
            {sensor: {}},
            [None, {sensor: []}],
        )

        # If there is no data to plot we just get the timeseries initialized figures back.
        raw_data = {sensor: {"frames": {}}}
        processed_data = [None, {sensor: [20e9, 21e9, 25e9, 26e9]}]
        rot_fig, trans_fig = init_pose_graph_figures(
            sensor, PoseType.Initial, raw_data, processed_data
        )
        # Check that the data is empty but the range is set for both figures.
        self.assertIsInstance(rot_fig, go.Figure)
        self.assertEqual(len(rot_fig["data"]), 0)
        self.assertEqual(rot_fig["layout"]["xaxis"]["range"], (0.0, 10.0))
        self.assertIsInstance(trans_fig, go.Figure)
        self.assertEqual(len(trans_fig["data"]), 0)
        self.assertEqual(trans_fig["layout"]["xaxis"]["range"], (0.0, 10.0))

        raw_data = {
            sensor: {
                "frames": {
                    20e9: [0, 0, 0, 0, 0, 0],
                    21e9: [0, 0, 0, 0, 0, 0],
                    25e9: [0, 0, 0, 0, 0, 0],
                    26e9: [0, 0, 0, 0, 0, 0],
                }
            }
        }
        processed_data = [None, {sensor: [20e9, 21e9, 25e9, 26e9]}]
        rot_fig, trans_fig = init_pose_graph_figures(
            sensor, PoseType.Initial, raw_data, processed_data
        )
        print(rot_fig)
