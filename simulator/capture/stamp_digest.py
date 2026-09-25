"""Canonical receipt hashes for ordered ROS header-stamp sequences."""

from __future__ import annotations

import hashlib


def stamp_sequence_sha256(stamps_s) -> str:
    digest = hashlib.sha256()
    for stamp_s in stamps_s:
        stamp_ns = int(round(float(stamp_s) * 1_000_000_000.0))
        digest.update(stamp_ns.to_bytes(8, byteorder="little", signed=True))
    return digest.hexdigest()
