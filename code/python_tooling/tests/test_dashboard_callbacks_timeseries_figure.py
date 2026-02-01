import unittest

import plotly.graph_objects as go

from dashboard.callbacks.timeseries_figure import (
    make_r3_timeseries_annotation_clientside_callback,
    plot_two_common_r3_timeseries,
)
from dashboard.tools.plot_r3_timeseries import R3TimeseriesFigureConfig as DefaultConfig
from database.types import PoseType, SensorType


class TestDashboardCallbacksTimeseriesFigure(unittest.TestCase):
    def test_build_timeseries_figures(self):
        # Unknown sensor type causes throw:
        self.assertRaises(
            RuntimeError,
            plot_two_common_r3_timeseries,
            None,
            None,
            "random_sensor",
            None,
            None,
            None,
        )

        # Data frames are empty but the raw timestamps are present - returns two empty but configured figures
        timestamps_ns = [20e8, 21e9, 22e9, 23e9, 24e9, 25e9]
        fig1, fig2 = plot_two_common_r3_timeseries(
            timestamps_ns, {}, SensorType.Imu, None, None, None
        )
        self.assertIsInstance(fig1, go.Figure)
        self.assertEqual(len(fig1["data"]), 0)
        self.assertIsInstance(fig2, go.Figure)
        self.assertEqual(len(fig2["data"]), 0)

        # Plot four data points - results in two figures with three traces each containing four points :)
        camera_frames = {
            21e9: {"poses": {PoseType.Initial: [0, 0, 0, 0, 0, 0]}},
            22e9: {"poses": {PoseType.Initial: [0, 0, 0, 0, 0, 0]}},
            23e9: {"poses": {PoseType.Initial: [0, 0, 0, 0, 0, 0]}},
            24e9: {"poses": {PoseType.Initial: [0, 0, 0, 0, 0, 0]}},
        }
        fig1, fig2 = plot_two_common_r3_timeseries(
            timestamps_ns,
            camera_frames,
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
            all(len(t.x) == len(t.y) == 4 for t in fig1_traces)
        )  # Four data points plotted
        # Fig2
        self.assertIsInstance(fig2, go.Figure)
        fig2_traces = fig2["data"]
        self.assertEqual(len(fig2_traces), 3)
        self.assertTrue(all(len(t.x) == len(t.y) == 4 for t in fig2_traces))

    def test_make_r3_timeseries_annotation_clientside_callback(self):
        clientside_callback = make_r3_timeseries_annotation_clientside_callback(
            SensorType.Camera
        )
        self.assertIn(f'metadata[1]["camera"]', clientside_callback)

        clientside_callback = make_r3_timeseries_annotation_clientside_callback(
            SensorType.Imu
        )
        self.assertIn(f'metadata[1]["imu"]', clientside_callback)
