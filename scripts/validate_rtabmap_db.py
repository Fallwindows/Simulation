"""Validate that a backed-up RTAB-Map database contains mapped nodes."""

from __future__ import annotations

import argparse
import sqlite3


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("database")
    args = parser.parse_args()
    with sqlite3.connect(args.database) as connection:
        node_count = int(connection.execute("select count(*) from Node").fetchone()[0])
    if node_count <= 0:
        raise SystemExit("RTAB-Map database contains no Node rows")
    print(node_count)


if __name__ == "__main__":
    main()
