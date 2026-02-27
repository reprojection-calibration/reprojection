import os
import sqlite3
import tempfile
import unittest

from database.load_extracted_targets import load_extracted_targets_df
from database.load_images import load_images_df
from database.load_imu_data import load_imu_data_df
from database.load_poses import load_poses_df
from database.load_reprojection_errors import load_reprojection_errors_df
from database.sql_statement_loading import load_sql


def execute_sql(sql_statement, db_path):
    conn = sqlite3.connect(db_path)

    cursor = conn.cursor()
    cursor.execute(sql_statement)
    conn.commit()

    conn.close()


class TestDatabaseDatabaseConnections(unittest.TestCase):
    # NOTE(Jack): This is a preemptive attempt to make it possible to execute the test locally or in the docker. Let's
    # see if this works or we just end up removing it later :)
    @classmethod
    def setUpClass(self):
        self.db_path = os.getenv(
            "DB_PATH", "/temporary/code/test_data/dataset-calib-imu4_512_16.db3"
        )

    def test_load_images_df(self):
        df = load_images_df("nonexistent.db3")
        self.assertIsNone(df)

        df = load_images_df(self.db_path)
        self.assertEqual(df.shape, (1758, 3))

    def test_load_extracted_targets_df(self):
        df = load_extracted_targets_df("nonexistent.db3")
        self.assertIsNone(df)

        df = load_extracted_targets_df(self.db_path)
        self.assertEqual(df.shape, (1758, 3))

    def test_load_poses_df(self):
        df = load_poses_df("nonexistent.db3")
        self.assertIsNone(df)

        # Checked in test database does not hold calculated values like poses
        df = load_poses_df(self.db_path)
        self.assertIsNone(df)

        # Create the table in a temporary database and test that at least a dataframe with no data rows is loaded.
        with tempfile.NamedTemporaryFile(suffix=".db3") as db_path_tmp:
            db_path = db_path_tmp.name
            execute_sql(load_sql("poses_table.sql"), db_path)

            df = load_poses_df(db_path)
            self.assertEqual(df.shape, (0, 9))

    def test_load_reprojection_errors_df(self):
        df = load_reprojection_errors_df("nonexistent.db3")
        self.assertIsNone(df)

        df = load_reprojection_errors_df(self.db_path)
        self.assertIsNone(df)

        with tempfile.NamedTemporaryFile(suffix=".db3") as db_path_tmp:
            db_path = db_path_tmp.name
            execute_sql(load_sql("reprojection_error_table.sql"), db_path)

            df = load_reprojection_errors_df(db_path)
            self.assertEqual(df.shape, (0, 4))

    def test_load_imu_data_df(self):
        df = load_imu_data_df("nonexistent.db3")
        self.assertIsNone(df)

        df = load_imu_data_df(self.db_path)
        self.assertEqual(df.shape, (8770, 8))


if __name__ == "__main__":
    unittest.main()
