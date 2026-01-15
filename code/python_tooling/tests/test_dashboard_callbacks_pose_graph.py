import unittest

from dashboard.callbacks.pose_graph import init_pose_graph_figures

from database.types import PoseType


class TestDashboardCallbacksPoseGraph(unittest.TestCase):
    def test_init_pose_graph_figures(self):
        rot_fig, trans_fig = init_pose_graph_figures(None, None, None)
        self.assertEqual(rot_fig, {})
        self.assertEqual(trans_fig, {})

        sensor = "/cam0/image_raw"
        rot_fig, trans_fig = init_pose_graph_figures(sensor, PoseType.Initial, {'/not/cam0/image_raw': None})
        self.assertEqual(rot_fig, {})
        self.assertEqual(trans_fig, {})

        raw_data = {sensor: {}}
        self.assertRaises(RuntimeError, init_pose_graph_figures, sensor, PoseType.Initial, raw_data)

        raw_data[sensor] = {'frames': None}
        rot_fig, trans_fig = init_pose_graph_figures(sensor, PoseType.Initial, raw_data)
        self.assertEqual(rot_fig, {})
        self.assertEqual(trans_fig, {})
