from __future__ import annotations

import sqlite3
import subprocess
import sys
import tempfile
import unittest
from contextlib import closing
from pathlib import Path


class ValidateRtabmapDatabaseTests(unittest.TestCase):
    def _run(self, database: Path) -> subprocess.CompletedProcess[str]:
        root = Path(__file__).resolve().parents[1]
        return subprocess.run(
            [sys.executable, str(root / "scripts" / "validate_rtabmap_db.py"), str(database)],
            cwd=root,
            capture_output=True,
            text=True,
            timeout=10,
        )

    def test_accepts_integral_database_with_nodes(self):
        with tempfile.TemporaryDirectory() as directory:
            database = Path(directory) / "rtabmap.db"
            with closing(sqlite3.connect(database)) as connection:
                connection.execute("create table Node(id integer primary key)")
                connection.execute("create table Data(id integer primary key, scan blob)")
                connection.executemany("insert into Node(id) values (?)", [(1,), (2,)])
                connection.executemany("insert into Data(id, scan) values (?, ?)", [(1, b"scan-a"), (2, b"scan-b")])
                connection.commit()
            completed = self._run(database)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertEqual(
                completed.stdout.strip(),
                '{"integrity": "ok", "node_count": 2, "scan_count": 2}',
            )

    def test_rejects_empty_and_malformed_databases(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            empty = root / "empty.db"
            with closing(sqlite3.connect(empty)) as connection:
                connection.execute("create table Node(id integer primary key)")
                connection.execute("create table Data(id integer primary key, scan blob)")
                connection.commit()
            empty_result = self._run(empty)
            self.assertNotEqual(empty_result.returncode, 0)
            self.assertIn("has 0 Node rows; expected at least 2", empty_result.stderr)

            malformed = root / "malformed.db"
            malformed.write_bytes(b"not a sqlite database")
            malformed_result = self._run(malformed)
            self.assertNotEqual(malformed_result.returncode, 0)
            self.assertIn("file is not a database", malformed_result.stderr)


if __name__ == "__main__":
    unittest.main()
