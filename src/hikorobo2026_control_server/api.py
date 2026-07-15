from __future__ import annotations

import asyncio
import logging
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from hikorobo2026_control_server.config import Settings, settings
from hikorobo2026_control_server.csv_logger import CsvTelemetryLogger
from hikorobo2026_control_server.mavlink_bridge import MavlinkBridge
from hikorobo2026_control_server.state import SharedState

logger = logging.getLogger(__name__)

STATIC_DIR = Path(__file__).resolve().parent / "static"


class ParamSetBody(BaseModel):
    name: str = Field(min_length=1, max_length=16)
    value: float


class ParamReadBody(BaseModel):
    name: str | None = Field(default=None, max_length=16)
    index: int = -1


class StorageBody(BaseModel):
    action: int = Field(description="0=load, 1=save, 2=reset")


class VehicleOverrideBody(BaseModel):
    host: str
    port: int = Field(ge=1, le=65535)


class ConnectionManager:
    def __init__(self) -> None:
        self.active: set[WebSocket] = set()
        self._lock = asyncio.Lock()

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        async with self._lock:
            self.active.add(websocket)

    async def disconnect(self, websocket: WebSocket) -> None:
        async with self._lock:
            self.active.discard(websocket)

    async def broadcast(self, payload: dict[str, Any]) -> None:
        async with self._lock:
            sockets = list(self.active)
        stale: list[WebSocket] = []
        for ws in sockets:
            try:
                await ws.send_json(payload)
            except Exception:
                stale.append(ws)
        for ws in stale:
            await self.disconnect(ws)


def create_app(
    app_settings: Settings | None = None,
    shared: SharedState | None = None,
    bridge: MavlinkBridge | None = None,
) -> FastAPI:
    cfg = app_settings or settings
    state = shared or SharedState()
    mav = bridge or MavlinkBridge(cfg, state)
    ws_manager = ConnectionManager()
    csv_logger = CsvTelemetryLogger(cfg.csv_log_dir)
    heartbeat_task: asyncio.Task | None = None
    last_csv_write_mono = 0.0
    last_ws_push_mono = 0.0

    async def on_telemetry(payload: dict[str, Any]) -> None:
        nonlocal last_csv_write_mono, last_ws_push_mono
        now = time.monotonic()
        if csv_logger.active and (now - last_csv_write_mono) >= 0.1:
            csv_logger.write(payload)
            last_csv_write_mono = now
        if (now - last_ws_push_mono) >= 0.05:
            last_ws_push_mono = now
            await ws_manager.broadcast(payload)

    @asynccontextmanager
    async def lifespan(_app: FastAPI):
        nonlocal heartbeat_task
        mav.add_listener(on_telemetry)
        await mav.start()

        async def gcs_heartbeat_loop() -> None:
            while True:
                try:
                    mav.send_heartbeat()
                except Exception:
                    logger.debug("GCS heartbeat skipped", exc_info=True)
                await asyncio.sleep(1.0)

        heartbeat_task = asyncio.create_task(gcs_heartbeat_loop())
        logger.info("HTTP UI on http://127.0.0.1:%s (bind %s)", cfg.http_port, cfg.http_host)
        try:
            yield
        finally:
            if heartbeat_task is not None:
                heartbeat_task.cancel()
                try:
                    await heartbeat_task
                except asyncio.CancelledError:
                    pass
            csv_logger.stop()
            await mav.stop()

    app = FastAPI(title="hikorobo2026 control server", lifespan=lifespan)
    app.state.settings = cfg
    app.state.shared = state
    app.state.bridge = mav
    app.state.csv_logger = csv_logger

    @app.get("/api/health")
    async def health() -> dict[str, Any]:
        snap = state.snapshot()
        return {
            "ok": True,
            "udp_listen_port": cfg.udp_listen_port,
            "vehicle": (
                {"host": snap.vehicle_addr[0], "port": snap.vehicle_addr[1]}
                if snap.vehicle_addr
                else {"host": cfg.vehicle_host, "port": cfg.vehicle_port}
            ),
            "connected": snap.to_public_dict()["connected"],
        }

    @app.get("/api/telemetry")
    async def get_telemetry() -> dict[str, Any]:
        return state.snapshot().to_public_dict()

    @app.get("/api/parameters")
    async def get_parameters() -> dict[str, Any]:
        return {"parameters": state.snapshot().to_public_dict()["parameters"]}

    @app.post("/api/parameters/list")
    async def request_param_list() -> dict[str, str]:
        mav.request_parameter_list()
        return {"status": "requested"}

    @app.post("/api/parameters/read")
    async def request_param_read(body: ParamReadBody) -> dict[str, str]:
        if body.name is None and body.index < 0:
            raise HTTPException(status_code=400, detail="name or index required")
        mav.request_parameter_read(name=body.name, index=body.index)
        return {"status": "requested"}

    @app.post("/api/parameters/set")
    async def set_param(body: ParamSetBody) -> dict[str, Any]:
        mav.set_parameter(body.name, body.value)
        return {"status": "sent", "name": body.name, "value": body.value}

    @app.post("/api/storage")
    async def storage(body: StorageBody) -> dict[str, Any]:
        if body.action not in (0, 1, 2):
            raise HTTPException(status_code=400, detail="action must be 0, 1, or 2")
        mav.send_preflight_storage(body.action)
        return {"status": "sent", "action": body.action}

    @app.post("/api/vehicle")
    async def override_vehicle(body: VehicleOverrideBody) -> dict[str, Any]:
        def mutate(telemetry) -> None:
            telemetry.vehicle_addr = (body.host, body.port)

        state.update(mutate)
        return {"status": "ok", "host": body.host, "port": body.port}

    @app.get("/api/csv/status")
    async def csv_status() -> dict[str, Any]:
        return {"active": csv_logger.active, "path": csv_logger.path}

    @app.post("/api/csv/start")
    async def csv_start() -> dict[str, Any]:
        path = csv_logger.start()
        return {"active": True, "path": path}

    @app.post("/api/csv/stop")
    async def csv_stop() -> dict[str, Any]:
        csv_logger.stop()
        return {"active": False, "path": None}

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket) -> None:
        await ws_manager.connect(websocket)
        try:
            await websocket.send_json(state.snapshot().to_public_dict())
            while True:
                message = await websocket.receive()
                if message.get("type") == "websocket.disconnect":
                    break
        except WebSocketDisconnect:
            pass
        finally:
            await ws_manager.disconnect(websocket)

    if STATIC_DIR.exists():
        app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

        @app.get("/")
        async def index() -> FileResponse:
            return FileResponse(STATIC_DIR / "index.html")

    return app
