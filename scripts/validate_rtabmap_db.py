"""Validate that a backed-up RTAB-Map database contains mapped nodes."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sqlite3


def _stamp_sequence_sha256(stamps_s: list[float]) -> str:
    digest = hashlib.sha256()
    for stamp_s in stamps_s:
        stamp_ns = int(round(stamp_s * 1_000_000_000.0))
        digest.update(stamp_ns.to_bytes(8, byteorder="little", signed=True))
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("database")
    parser.add_argument("--minimum-node-stamp", type=float)
    parser.add_argument("--scan-period-seconds", type=float, default=0.0)
    parser.add_argument("--expected-node-count", type=int)
    parser.add_argument("--expected-stamp-sha256")
    args = parser.parse_args()
    with sqlite3.connect(args.database) as connection:
        node_stamps = [float(row[0]) for row in connection.execute("select stamp from Node order by stamp")]
        integrity = str(connection.execute("pragma integrity_check").fetchone()[0])
    node_count = len(node_stamps)
    last_node_stamp = node_stamps[-1] if node_stamps else None
    if node_count <= 0:
        raise SystemExit("RTAB-Map database contains no Node rows")
    if integrity.lower() != "ok":
        raise SystemExit(f"RTAB-Map database integrity check failed: {integrity}")
    if not all(math.isfinite(stamp) for stamp in node_stamps):
        raise SystemExit("RTAB-Map database contains a non-finite Node stamp")
    timestamps_strictly_increasing = all(
        current > previous for previous, current in zip(node_stamps, node_stamps[1:])
    )
    if not timestamps_strictly_increasing:
        raise SystemExit("RTAB-Map database Node stamps are not unique and strictly increasing")
    node_stamp_sha256 = _stamp_sequence_sha256(node_stamps)
    if (args.expected_node_count is None) != (args.expected_stamp_sha256 is None):
        raise SystemExit("expected node count and stamp SHA-256 must be provided together")
    if args.expected_node_count is not None:
        expected_stamp_sha256 = str(args.expected_stamp_sha256).lower()
        if args.expected_node_count <= 0:
            raise SystemExit("expected node count must be positive")
        if len(expected_stamp_sha256) != 64 or any(character not in "0123456789abcdef" for character in expected_stamp_sha256):
            raise SystemExit("expected stamp SHA-256 must be 64 lowercase hexadecimal characters")
        if node_count != args.expected_node_count:
            raise SystemExit(f"RTAB-Map database Node count {node_count} does not match captured LiDAR count {args.expected_node_count}")
        if node_stamp_sha256 != expected_stamp_sha256:
            raise SystemExit(
                f"RTAB-Map database Node stamp sequence {node_stamp_sha256} does not match "
                f"captured LiDAR sequence {expected_stamp_sha256}"
            )
    if args.minimum_node_stamp is not None:
        if last_node_stamp is None or float(last_node_stamp) < args.minimum_node_stamp - args.scan_period_seconds - 1e-3:
            raise SystemExit(f"RTAB-Map database ends before required sensor stamp: {last_node_stamp} < {args.minimum_node_stamp}")
    print(json.dumps({
        "node_count": node_count,
        "node_stamp_sha256": node_stamp_sha256,
        "node_timestamps_strictly_increasing": timestamps_strictly_increasing,
        "last_node_stamp_s": last_node_stamp,
        "integrity_check": integrity,
    }, sort_keys=True))


if __name__ == "__main__":
    main()
