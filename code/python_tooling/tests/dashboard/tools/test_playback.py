import json
import unittest
from tempfile import NamedTemporaryFile

from dashboard.tools.data_loading import load_database
from dashboard.tools.playback import synchronized_targets
from tests.test_fixture import construct_staged_visualization_db


class TestSynchronizedPlayback(unittest.TestCase):
    base = 1700000000000000001

    def data(self, samples, frames=None):
        targets, errors = [], []
        for asset_id, offsets in samples.items():
            for i, offset in enumerate(offsets):
                timestamp = str(self.base + offset)
                targets.append(
                    dict(
                        asset_id=asset_id,
                        step_id=20 + asset_id,
                        source_step_id=10 + asset_id,
                        timestamp_ns=timestamp,
                    )
                )
                errors.append(
                    dict(
                        asset_id=asset_id,
                        step_id=60,
                        source_step_id=20 + asset_id,
                        sample_timestamp_ns=timestamp,
                        frame_timestamp_ns=str(
                            self.base + (frames[asset_id][i] if frames else offset)
                        ),
                    )
                )
        return {"tables": {"extracted_targets": targets, "reprojection_errors": errors}}

    def test_stereo_groups_by_frame_not_sample_or_array_index(self):
        data = self.data(
            {1: [0, 30_000_000, 60_000_000], 2: [2_000_000, 58_000_000]},
            {1: [0, 30_000_000, 60_000_000], 2: [0, 60_000_000]},
        )
        # Distractors from a different extraction and a different result must not join.
        data["tables"]["extracted_targets"].append(
            dict(asset_id=2, step_id=99, timestamp_ns=str(self.base))
        )
        data["tables"]["reprojection_errors"].append(
            dict(data["tables"]["reprojection_errors"][0], step_id=61)
        )
        frames = synchronized_targets(data, [1, 2], 60, "multi_cam", 0)
        self.assertEqual(
            [f["timestamp_ns"] for f in frames],
            [str(self.base + n) for n in [0, 30_000_000, 60_000_000]],
        )
        self.assertEqual(
            [set(f["cameras"]) for f in frames], [{"1", "2"}, {"1"}, {"1", "2"}]
        )
        self.assertEqual(
            frames[0]["cameras"]["2"]["timestamp_ns"], str(self.base + 2_000_000)
        )
        self.assertEqual(
            frames[2]["cameras"]["2"]["timestamp_ns"], str(self.base + 58_000_000)
        )

    def test_spline_uses_nearest_samples_with_bounded_skew(self):
        # Different frame rates, dropped samples, and a camera starting later.
        data = self.data(
            {
                1: [0, 30_000_000, 60_000_000, 90_000_000],
                2: [4_000_000, 64_000_000],
                3: [59_000_000],
            }
        )
        frames = synchronized_targets(data, [1, 2, 3], 60, "cam_imu", 5)
        self.assertEqual(len(frames), 7)
        by_time = {int(f["timestamp_ns"]) - self.base: f["cameras"] for f in frames}
        self.assertEqual(set(by_time[0]), {"1", "2"})
        self.assertEqual(set(by_time[30_000_000]), {"1"})
        self.assertEqual(set(by_time[60_000_000]), {"1", "2", "3"})
        self.assertEqual(set(by_time[90_000_000]), {"1"})
        self.assertEqual(
            by_time[64_000_000]["1"]["timestamp_ns"], str(self.base + 60_000_000)
        )
        exact = synchronized_targets(data, [1, 2, 3], 60, "cam_imu", 0)
        self.assertTrue(all(len(f["cameras"]) == 1 for f in exact))

    def test_nanosecond_precision_and_deterministic_ties(self):
        data = self.data({1: [1], 2: [0, 2]})
        frames = synchronized_targets(data, [1, 2], 60, "cam_imu", 0.000001)
        self.assertEqual(frames[1]["cameras"]["2"]["timestamp_ns"], str(self.base))
        self.assertEqual(len(synchronized_targets(data, [1, 2], 60, "multi_cam")), 3)

    def test_empty_and_detection_only_data(self):
        self.assertEqual(synchronized_targets({}, [1, 2], None, "cam_imu"), [])
        data = self.data({1: [0], 2: [3_000_000]})
        data["tables"]["reprojection_errors"] = []
        frames = synchronized_targets(data, [1, 2], None, "cam_imu")
        self.assertEqual(len(frames), 2)
        self.assertEqual(set(frames[0]["cameras"]), {"1", "2"})

    def test_database_frame_timestamps_survive_json_exactly(self):
        with NamedTemporaryFile(suffix=".db3") as database:
            construct_staged_visualization_db(database.name)
            data, _ = load_database(database.name, 1)
        data = json.loads(json.dumps(data))
        row = data["tables"]["reprojection_errors"][0]
        self.assertEqual(row["frame_timestamp_ns"], str(self.base))
        for stage, step_id in [("multi_cam", 71), ("cam_imu", 60)]:
            frames = synchronized_targets(data, [1, 2], step_id, stage)
            self.assertEqual(len(frames), 2)
            self.assertEqual(set(frames[0]["cameras"]), {"1", "2"})
