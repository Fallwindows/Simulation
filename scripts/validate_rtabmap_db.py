"""Validate that a backed-up RTAB-Map database contains mapped nodes."""

from __future__ import annotations

import argparse
import json
import sqlite3


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("database")
    parser.add_argument("--minimum-node-stamp", type=float)
    parser.add_argument("--scan-period-seconds", type=float, default=0.0)
    args = parser.parse_args()
    with sqlite3.connect(args.database) as connection:
        node_count = int(connection.execute("select count(*) from Node").fetchone()[0])
        last_node_stamp = connection.execute("select max(stamp) from Node").fetchone()[0]
        integrity = str(connection.execute("pragma integrity_check").fetchone()[0])
    if node_count <= 0:
        raise SystemExit("RTAB-Map database contains no Node rows")
    if integrity.lower() != "ok":
        raise SystemExit(f"RTAB-Map database integrity check failed: {integrity}")
    if args.minimum_node_stamp is not None:
        if last_node_stamp is None or float(last_node_stamp) < args.minimum_node_stamp - args.scan_period_seconds - 1e-3:
            raise SystemExit(f"RTAB-Map database ends before required sensor stamp: {last_node_stamp} < {args.minimum_node_stamp}")
    print(json.dumps({"node_count": node_count, "last_node_stamp_s": last_node_stamp, "integrity_check": integrity}, sort_keys=True))


if __name__ == "__main__":
    main()
