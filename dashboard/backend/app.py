"""FastAPI dashboard boundary fed by ROS-facing caches.

The module can be imported without FastAPI/rclpy. On a runtime machine, a ROS
bridge should call the state update methods from subscriptions; the browser
never reads Isaac internals directly.
"""

from __future__ import annotations

import asyncio
import time
from pathlib import Path
from typing import Iterator

from dashboard.backend.state import DashboardState

try:
    from fastapi import FastAPI, WebSocket
    from fastapi.responses import FileResponse, Response, StreamingResponse
    from fastapi.staticfiles import StaticFiles
except ImportError:  # pragma: no cover - exercised only on a minimal host
    FastAPI = None  # type: ignore[assignment]
    WebSocket = object  # type: ignore[assignment,misc]
    FileResponse = Response = StreamingResponse = StaticFiles = None  # type: ignore[assignment]


WEB_ROOT = Path(__file__).resolve().parents[1] / "web"


def _mjpeg_stream(state: DashboardState) -> Iterator[bytes]:
    while True:
        frame = state.rgb_jpeg
        if frame:
            yield b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n"
        time.sleep(1.0 / 30.0)


def create_app(state: DashboardState | None = None):
    if FastAPI is None:
        raise RuntimeError("FastAPI is required for the dashboard; install dashboard requirements")
    state = state or DashboardState()
    app = FastAPI(title="Grocery Aisle Simulation Dashboard")
    app.mount("/static", StaticFiles(directory=WEB_ROOT), name="static")

    @app.get("/")
    def index():
        return FileResponse(WEB_ROOT / "index.html")

    @app.get("/app.js")
    def javascript():
        return FileResponse(WEB_ROOT / "app.js", media_type="text/javascript")

    @app.get("/api/health")
    def health():
        return {"ok": True, "service": "grocery_sim_dashboard"}

    @app.get("/api/status")
    def status():
        return state.snapshot()["status"]

    @app.get("/api/metrics")
    def metrics():
        return state.snapshot()["metrics"]

    @app.get("/stream/rgb.mjpg")
    def rgb_stream():
        return StreamingResponse(_mjpeg_stream(state), media_type="multipart/x-mixed-replace; boundary=frame")

    async def points_socket(websocket: WebSocket, kind: str) -> None:
        await websocket.accept()
        while True:
            await websocket.send_json({"kind": kind, "points": state.decimated_points(kind, 50000)})
            await asyncio.sleep(0.2)

    @app.websocket("/ws/lidar")
    async def lidar_socket(websocket: WebSocket):
        await points_socket(websocket, "lidar")

    @app.websocket("/ws/map")
    async def map_socket(websocket: WebSocket):
        await points_socket(websocket, "map")

    app.state.dashboard = state
    return app


app = create_app() if FastAPI is not None else None
