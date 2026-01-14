import unittest

from dashboard.time_handling import calculate_ticks_from_timestamps, extract_timestamps_and_poses_sorted, \
    timestamps_to_elapsed_seconds
from database.types import PoseType


class TestDashboardTimeHandling(unittest.TestCase):
    def test_extract_timestamps_and_poses_sorted(self):
        # Purposely out of order, and purposely have three initial poses but only two optimized poses.
        frames = {
            1: {'poses': {PoseType.Initial: [0, 1, 2, 3, 4, 5, 6.1], PoseType.Optimized: [0, 1, 2, 3, 4, 5, -6.1]}},
            3: {'poses': {PoseType.Initial: [0, 1, 2, 3, 4, 5, 6.3], PoseType.Optimized: [0, 1, 2, 3, 4, 5, -6.3]}},
            2: {'poses': {PoseType.Initial: [0, 1, 2, 3, 4, 5, 6.2]}}}

        # PoseType.Initial
        timestamps, poses = extract_timestamps_and_poses_sorted(frames, PoseType.Initial)
        self.assertEqual(len(timestamps), 3)
        self.assertEqual(len(poses), 3)

        self.assertEqual(timestamps, [1, 2, 3])
        self.assertEqual(poses, [[0, 1, 2, 3, 4, 5, 6.1], [0, 1, 2, 3, 4, 5, 6.2], [0, 1, 2, 3, 4, 5, 6.3]])

        # PoseType.Optimized
        timestamps, poses = extract_timestamps_and_poses_sorted(frames, PoseType.Optimized)
        self.assertEqual(len(timestamps), 2)
        self.assertEqual(len(poses), 2)

        self.assertEqual(timestamps, [1, 3])
        self.assertEqual(poses, [[0, 1, 2, 3, 4, 5, -6.1], [0, 1, 2, 3, 4, 5, -6.3]])
