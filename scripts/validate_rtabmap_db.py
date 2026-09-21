"""Validate that a backed-up RTAB-Map database contains mapped nodes."""

from __future__ import annotations

import argparse
import json
import sqlite3
from contextlib import closing


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("database")
    parser.add_argument("--min-nodes", type=int, default=2)
    parser.add_argument("--min-scans", type=int, default=2)
    args = parser.parse_args()
    with closing(sqlite3.connect(args.database)) as connection:
        integrity = str(connection.execute("pragma integrity_check").fetchone()[0])
        if integrity != "ok":
            raise SystemExit(f"RTAB-Map database integrity check failed: {integrity}")
        node_count = int(connection.execute("select count(*) from Node").fetchone()[0])
        scan_count = int(
            connection.execute("select count(*) from Data where scan is not null and length(scan) > 0").fetchone()[0]
        )
    if node_count < args.min_nodes:
        raise SystemExit(f"RTAB-Map database has {node_count} Node rows; expected at least {args.min_nodes}")
    if scan_count < args.min_scans:
        raise SystemExit(f"RTAB-Map database has {scan_count} scan rows; expected at least {args.min_scans}")
    print(json.dumps({"integrity": "ok", "node_count": node_count, "scan_count": scan_count}, sort_keys=True))


if __name__ == "__main__":
    main()
