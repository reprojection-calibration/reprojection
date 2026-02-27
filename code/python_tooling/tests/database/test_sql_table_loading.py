import os
import sqlite3
import tempfile
import unittest

from database.sql_statement_loading import load_sql
from database.sql_table_loading import (
    load_extracted_targets_table,
    load_images_table,
    load_imu_data_table,
    load_poses_table,
    load_reprojection_errors_table,
)


def execute_sql(sql_statement, db_path):
    conn = sqlite3.connect(db_path)

    cursor = conn.cursor()
    cursor.execute(sql_statement)
    conn.commit()

    conn.close()


class TestDatabaseSqlTableLoading(unittest.TestCase):
    # NOTE(Jack): This is a preemptive attempt to make it possible to execute the test locally or in the docker. Let's
    # see if this works or we just end up removing it later :)
    @classmethod
    def setUpClass(self):
        self.db_path = os.getenv(
            "DB_PATH", "/temporary/code/test_data/dataset-calib-imu4_512_16.db3"
        )

    def test_load_images_table(self):
        table = load_images_table("nonexistent.db3")
        self.assertIsNone(table)

        table = load_images_table(self.db_path)
        self.assertEqual(table.shape, (1758, 3))

    def test_load_extracted_targets_table(self):
        table = load_extracted_targets_table("nonexistent.db3")
        self.assertIsNone(table)

        table = load_extracted_targets_table(self.db_path)
        self.assertEqual(table.shape, (1758, 3))

    def test_load_poses_table(self):
        table = load_poses_table("nonexistent.db3")
        self.assertIsNone(table)

        # Checked in test database does not hold tables for calculated values like poses
        table = load_poses_table(self.db_path)
        self.assertIsNone(table)

        # Create the table in a temporary database and test that at least a dataframe with no data rows is loaded.
        with tempfile.NamedTemporaryFile(suffix=".db3") as db_path_tmp:
            db_path = db_path_tmp.name
            execute_sql(load_sql("poses_table.sql"), db_path)

            table = load_poses_table(db_path)
            self.assertEqual(table.shape, (0, 9))

    def test_load_reprojection_errors_table(self):
        table = load_reprojection_errors_table("nonexistent.db3")
        self.assertIsNone(table)

        table = load_reprojection_errors_table(self.db_path)
        self.assertIsNone(table)

        with tempfile.NamedTemporaryFile(suffix=".db3") as db_path_tmp:
            db_path = db_path_tmp.name
            execute_sql(load_sql("reprojection_error_table.sql"), db_path)

            table = load_reprojection_errors_table(db_path)
            self.assertEqual(table.shape, (0, 4))

    def test_load_imu_data_table(self):
        table = load_imu_data_table("nonexistent.db3")
        self.assertIsNone(table)

        table = load_imu_data_table(self.db_path)
        self.assertEqual(table.shape, (8770, 8))


if __name__ == "__main__":
    unittest.main()
