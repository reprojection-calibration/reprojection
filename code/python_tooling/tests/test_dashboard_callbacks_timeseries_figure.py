import unittest

import plotly.graph_objects as go
from dashboard_reference_data import full_data

from dashboard.callbacks.r3_timeseries_figure import (
    build_r6_timeseries_figures,
    make_timeseries_annotation_clientside_callback,
)
from dashboard.tools.r3_timeseries_figure import (
    R3TimeseriesFigureConfig as DefaultConfig,
)
from database.types import PoseType, SensorType


class TestDashboardCallbacksTimeseriesFigure(unittest.TestCase):

    def test_build_r6_timeseries_figures_full_data(self):
        test_data = full_data()

        fig1, fig2 = build_r6_timeseries_figures(
            test_data.timestamps[SensorType.Camera]["/cam0/image_raw"],
            test_data.raw_camera_data["/cam0/image_raw"]["frames"],
            SensorType.Camera,
            DefaultConfig(),
            DefaultConfig(),
            PoseType.Initial,
        )

        self.assertIsInstance(fig1, go.Figure)
        self.assertEqual(len(fig1["data"]), 0)
        self.assertIsInstance(fig2, go.Figure)
        self.assertEqual(len(fig2["data"]), 0)

    def test_build_r6_timeseries_figures(self):
        # We add some pose data to check that the plotting works like we expect
        test_data = full_data()
        test_data.timestamps[SensorType.Camera]["/cam0/image_raw"] = [0, 1, 2, 3]
        test_data.raw_camera_data["/cam0/image_raw"]["frames"] = {
            0: {"poses": {PoseType.Initial: [0, 0, 0, 0, 0, 0]}},
            1: {"poses": {PoseType.Initial: [0, 0, 0, 0, 0, 0]}},
            2: {"poses": {PoseType.Initial: [0, 0, 0, 0, 0, 0]}},
            3: {"poses": {PoseType.Initial: [0, 0, 0, 0, 0, 0]}},
        }

        # Plot four data points - results in two figures with three traces each containing four points :)
        fig1, fig2 = build_r6_timeseries_figures(
            test_data.timestamps[SensorType.Camera]["/cam0/image_raw"],
            test_data.raw_camera_data["/cam0/image_raw"]["frames"],
            SensorType.Camera,
            DefaultConfig(),
            DefaultConfig(),
            PoseType.Initial,
        )

        # Fig1
        self.assertIsInstance(fig1, go.Figure)
        fig1_traces = fig1["data"]
        self.assertEqual(len(fig1_traces), 3)  # Three traces (x,y,z)
        self.assertTrue(
            all(len(trace.x) == len(trace.y) == 4 for trace in fig1_traces)
        )  # Four data points plotted

        # Fig2
        self.assertIsInstance(fig2, go.Figure)
        fig2_traces = fig2["data"]
        self.assertEqual(len(fig2_traces), 3)
        self.assertTrue(all(len(trace.x) == len(trace.y) == 4 for trace in fig2_traces))

    def test_make_timeseries_annotation_clientside_callback(self):
        clientside_callback = make_timeseries_annotation_clientside_callback(
            SensorType.Camera
        )
        self.assertIn(f'metadata[1]["camera"]', clientside_callback)

        clientside_callback = make_timeseries_annotation_clientside_callback(
            SensorType.Imu
        )
        self.assertIn(f'metadata[1]["imu"]', clientside_callback)
