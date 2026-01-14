import unittest

from dashboard.tools.time_handling import (
    calculate_ticks_from_timestamps,
    extract_timestamps_and_poses_sorted,
    timestamps_to_elapsed_seconds,
)
from database.types import PoseType


class TestDashboardTimeHandling(unittest.TestCase):
    def test_extract_timestamps_and_poses_sorted(self):
        # Purposely out of order, and purposely have three initial poses but only two optimized poses.
        frames = {
            1: {
                "poses": {
                    PoseType.Initial: [0, 1, 2, 3, 4, 5, 6.1],
                    PoseType.Optimized: [0, 1, 2, 3, 4, 5, -6.1],
                }
            },
            3: {
                "poses": {
                    PoseType.Initial: [0, 1, 2, 3, 4, 5, 6.3],
                    PoseType.Optimized: [0, 1, 2, 3, 4, 5, -6.3],
                }
            },
            2: {"poses": {PoseType.Initial: [0, 1, 2, 3, 4, 5, 6.2]}},
        }

        # PoseType.Initial
        timestamps, poses = extract_timestamps_and_poses_sorted(
            frames, PoseType.Initial
        )
        self.assertEqual(len(timestamps), 3)
        self.assertEqual(len(poses), 3)
        self.assertEqual(timestamps, [1, 2, 3])
        self.assertEqual(
            poses,
            [[0, 1, 2, 3, 4, 5, 6.1], [0, 1, 2, 3, 4, 5, 6.2], [0, 1, 2, 3, 4, 5, 6.3]],
        )

        # PoseType.Optimized
        timestamps, poses = extract_timestamps_and_poses_sorted(
            frames, PoseType.Optimized
        )
        self.assertEqual(len(timestamps), 2)
        self.assertEqual(len(poses), 2)
        self.assertEqual(timestamps, [1, 3])
        self.assertEqual(poses, [[0, 1, 2, 3, 4, 5, -6.1], [0, 1, 2, 3, 4, 5, -6.3]])

    def test_timestamps_to_elapsed_seconds(self):
        timestamps_ns = []
        elapsed_time_s = timestamps_to_elapsed_seconds(timestamps_ns)
        self.assertEqual(len(elapsed_time_s), 0)

        timestamps_ns = [0, 1e9, 2e9, 3e9]
        elapsed_time_s = timestamps_to_elapsed_seconds(timestamps_ns)
        self.assertEqual(len(elapsed_time_s), 4)
        self.assertEqual(elapsed_time_s, [0, 1, 2, 3])

        timestamps_ns = [10e9, 20e9, 30e9, 35e9]
        elapsed_time_s = timestamps_to_elapsed_seconds(timestamps_ns)
        self.assertEqual(elapsed_time_s, [0, 10, 20, 25])

    def test_calculate_ticks_from_timestamps(self):
        timestamps_ns = []
        tickvals_idx, tickvals_s, ticktext = calculate_ticks_from_timestamps(
            timestamps_ns
        )
        self.assertEqual(len(tickvals_idx), 0)
        self.assertEqual(len(tickvals_s), 0)
        self.assertEqual(len(ticktext), 0)

        # NOTE(Jack): Due to the simplified nature of the input test data all three outputs look very similar, but for
        # the general "real data" case that will not be! The tickvals_idx and tickvals_s will be on a completely
        # different scale/magnitude depending on the data frequency.
        timestamps_ns = [int(i * 1e9) for i in range(20)]
        tickvals_idx, tickvals_s, ticktext = calculate_ticks_from_timestamps(
            timestamps_ns, step=5
        )
        self.assertEqual(len(tickvals_idx), 4)
        self.assertEqual(len(tickvals_s), 4)
        self.assertEqual(len(ticktext), 4)
        self.assertEqual(tickvals_idx, [0, 5, 10, 15])
        self.assertEqual(tickvals_s, [0.0, 5.0, 10.0, 15.0])
        self.assertEqual(ticktext, ["0s", "5s", "10s", "15s"])

        tickvals_idx, tickvals_s, ticktext = calculate_ticks_from_timestamps(
            timestamps_ns, step=2
        )
        self.assertEqual(len(tickvals_idx), 10)
        self.assertEqual(len(tickvals_s), 10)
        self.assertEqual(len(ticktext), 10)
