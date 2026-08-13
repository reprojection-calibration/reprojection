import unittest
from pathlib import Path
from tempfile import NamedTemporaryFile, TemporaryDirectory

from dashboard.tools.data_loading import (
    load_database,
    refresh_database_list,
    refresh_sensor_list,
)
from database.calculate_metadata import count_data
from database.types import SensorType
from tests.test_fixture import construct_test_db


class TestDataLoading(unittest.TestCase):
    def test_refresh_database_list(self):
        with TemporaryDirectory() as tmp_dir:
            tmp_path = Path(tmp_dir)
            (tmp_path / "file_1.calib.db3").write_text("x")
            (tmp_path / "file_2.calib.db3").write_text("y")

            database_list, default_db = refresh_database_list(tmp_dir)

        self.assertEqual(
            database_list,
            [
                {
                    "label": "file_1.calib.db3",
                    "value": str(tmp_path / "file_1.calib.db3"),
                },
                {
                    "label": "file_2.calib.db3",
                    "value": str(tmp_path / "file_2.calib.db3"),
                },
            ],
        )

        self.assertEqual(default_db, str(tmp_path / "file_1.calib.db3"))

    def test_refresh_database_list_adversarial(self):
        db_paths = refresh_database_list(None)
        self.assertEqual(db_paths, ([], ""))

        db_paths = refresh_database_list("directory/that/does/not/exist/")
        self.assertEqual(db_paths, ([], ""))

    def test_load_database(self):
        with NamedTemporaryFile(suffix=".db3") as tmp:
            construct_test_db(tmp.name)

            raw_data, metadata = load_database(tmp.name, 1)

        gt_metadata = {"": {"measurements": {"images": 1}, "type": SensorType.Camera}}

        self.assertEqual(count_data(raw_data), gt_metadata)
        self.assertEqual(metadata, gt_metadata)

    def test_load_database_adversarial(self):
        result = load_database(None, 1)
        self.assertEqual(result, ({}, {}))

        self.assertRaises(
            FileNotFoundError, load_database, "file/that/does/not/exist.db3", 1
        )

    def test_refresh_sensor_list(self):
        metadata = {
            "/cam0/image_raw": {
                "measurements": {"images": 1},
                "type": SensorType.Camera,
            }
        }

        sensor_list, first_sensor = refresh_sensor_list(metadata)

        gt_sensor_list = [
            {
                "label": "/cam0/image_raw (SensorType.Camera)",
                "value": "/cam0/image_raw",
            }
        ]
        self.assertEqual(sensor_list, gt_sensor_list)
        self.assertEqual(first_sensor, gt_sensor_list[0]["value"])

    def test_refresh_sensor_list_adversarial(self):
        sensor_data = refresh_sensor_list(None)
        self.assertEqual(sensor_data, ([], ""))

        sensor_data = refresh_sensor_list({})
        self.assertEqual(sensor_data, ([], ""))
