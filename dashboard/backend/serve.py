"""Dashboard process entry point with an optional ROS subscription thread."""

from __future__ import annotations

import argparse
import threading

from dashboard.backend.app import create_app
from dashboard.backend.ros_bridge import RosDashboardBridge
from dashboard.backend.state import DashboardState


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--no-ros", action="store_true")
    args = parser.parse_args()
    state = DashboardState()
    if not args.no_ros:
        try:
            bridge = RosDashboardBridge(state)
        except RuntimeError as exc:
            print(f"ROS bridge unavailable; dashboard will serve empty caches: {exc}")
        else:
            threading.Thread(target=bridge.start, name="ros-dashboard-bridge", daemon=True).start()
    import uvicorn
    uvicorn.run(create_app(state), host=args.host, port=args.port)


if __name__ == "__main__":
    main()
