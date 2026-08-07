import os
import sqlite3
import unittest
from tempfile import NamedTemporaryFile

from database.sql_statement_loading import load_sql
from database.sql_table_loading import (
    load_calibration_database,
    load_table,
    load_table_blob,
)


def execute_sql(db_path, sql_query_file):
    conn = sqlite3.connect(db_path)

    cursor = conn.cursor()
    cursor.execute(sql_query_file)
    conn.commit()

    conn.close()


class TestDatabaseSqlTableLoading(unittest.TestCase):
    def test_load_table(self):
        self.assertRaises(
            FileNotFoundError, load_table, "nonexistent.db3", "nonexistent.sql"
        )

        with NamedTemporaryFile(suffix=".db3") as tmp:
            # Create a table so that we can load it.
            workflows_table = load_sql("workflows_table.sql")
            execute_sql(tmp.name, workflows_table)

            # Load the table.
            table = load_table(tmp.name, "workflows_select_all.sql")

            # Assert some of its properties.
            self.assertTrue(table.empty)
            self.assertEqual(list(table.columns), ["id", "type", "signature"])

    def test_load_table_blob(self):
        with NamedTemporaryFile(suffix=".db3") as tmp:
            # Create a table so that we can load it.
            extracted_targets_table = load_sql("extracted_targets_table.sql")
            execute_sql(tmp.name, extracted_targets_table)

            # Try to load the table with an improperly specified blob column id.
            self.assertRaises(
                KeyError,
                load_table_blob,
                tmp.name,
                "extracted_targets_select_all.sql",
                lambda value: value,
                "xyz",
            )

            # Load the table.
            table = load_table_blob(
                tmp.name, "extracted_targets_select_all.sql", lambda value: value
            )

            # Assert some of its properties.
            self.assertTrue(table.empty)
            self.assertEqual(
                list(table.columns), ["step_id", "asset_id", "timestamp_ns", "data"]
            )

    def test_load_calibration_database(self):
        with NamedTemporaryFile(suffix=".db3") as tmp:
            execute_sql(tmp.name, load_sql("workflow_assets_table.sql"))
            execute_sql(tmp.name, load_sql("workflow_steps_table.sql"))
            execute_sql(tmp.name, load_sql("workflows_table.sql"))

            db = load_calibration_database(tmp.name)

            self.assertEqual(
                list(db.keys()), ["workflow_assets", "workflow_steps", "workflows"]
            )


if __name__ == "__main__":
    unittest.main()
