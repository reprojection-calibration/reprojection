import os
import sqlite3
import unittest
from tempfile import NamedTemporaryFile

from database.sql_table_loading import load_table, load_table_blob
from database.sql_statement_loading import load_sql


def execute_sql(db_path, sql_query_file):
    conn = sqlite3.connect(db_path)

    cursor = conn.cursor()
    cursor.execute(sql_query_file)
    conn.commit()

    conn.close()


class TestDatabaseSqlTableLoading(unittest.TestCase):
    def test_load_table(self):
        self.assertRaises(FileNotFoundError, load_table, "nonexistent.db3", "nonexistent.sql")

        with NamedTemporaryFile(suffix=".db3") as tmp:
            # Create a table so that we can load it
            workflows_table = load_sql("workflows_table.sql")
            execute_sql(tmp.name, workflows_table)

            # Load the table
            table = load_table(tmp.name, "workflows_select_all.sql")

            # Assert some of its properties
            self.assertTrue(table.empty)
            self.assertEqual(list(table.columns), ["id", "type", "signature"])


if __name__ == "__main__":
    unittest.main()
