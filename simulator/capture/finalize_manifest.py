"""Command-line boundary for canonical capture-manifest finalization."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from simulator.capture.manifest import (
    finalize_capture_manifest,
    validate_capture_manifest_artifacts,
    validate_capture_manifest_hash,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    finalize_parser = subparsers.add_parser("finalize")
    finalize_parser.add_argument("--staging", required=True)
    finalize_parser.add_argument("--output", required=True)
    verify_parser = subparsers.add_parser("verify")
    verify_parser.add_argument("--manifest", required=True)
    args = parser.parse_args()
    if args.command == "finalize":
        manifest = finalize_capture_manifest(args.staging, args.output)
        result = {"status": "complete", "path": str(Path(args.output).resolve()), "capture_sha256": manifest["capture_sha256"]}
    else:
        path = Path(args.manifest)
        manifest = json.loads(path.read_text(encoding="utf-8"))
        digest = validate_capture_manifest_hash(manifest)
        validate_capture_manifest_artifacts(manifest, path.parent)
        result = {"status": "valid", "path": str(path.resolve()), "capture_sha256": digest}
    print(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()
