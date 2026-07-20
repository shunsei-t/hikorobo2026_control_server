from __future__ import annotations

import asyncio
import logging
import time
from collections.abc import Awaitable, Callable
from typing import Any

from pymavlink.dialects.v20 import common as mavlink2

from hikorobo2026_control_server.config import Settings
from hikorobo2026_control_server.state import (
    AttitudeSnapshot,
    AttitudeTargetSnapshot,
    HeartbeatSnapshot,
    ParameterValue,
    PressureSnapshot,
    RcChannelsSnapshot,
    SharedState,
    quat_to_euler_rad,
)

logger = logging.getLogger(__name__)

TelemetryListener = Callable[[dict[str, Any]], Awaitable[None] | None]


class MavlinkBridge:
    """Owns the MAVLink UDP socket used by the GCS."""

    def __init__(self, settings: Settings, shared: SharedState) -> None:
        self.settings = settings
        self.shared = shared
        self._transport: asyncio.DatagramTransport | None = None
        self._protocol: _MavlinkProtocol | None = None
        self._listeners: list[TelemetryListener] = []
        self._mav = mavlink2.MAVLink(
            file=None,
            srcSystem=settings.gcs_system_id,
            srcComponent=settings.gcs_component_id,
        )
        self._mav.robust_parsing = True

    def add_listener(self, listener: TelemetryListener) -> None:
        self._listeners.append(listener)

    async def start(self) -> None:
        loop = asyncio.get_running_loop()
        host = self.settings.udp_listen_host
        port = self.settings.udp_listen_port
        try:
            transport, protocol = await loop.create_datagram_endpoint(
                lambda: _MavlinkProtocol(self),
                local_addr=(host, port),
            )
        except OSError as exc:
            winerror = getattr(exc, "winerror", None)
            errno = getattr(exc, "errno", None)
            address_in_use = winerror == 10048 or errno in {48, 98}
            if address_in_use:
                raise RuntimeError(
                    f"UDP port {host}:{port} is already in use. "
                    "Stop the other process (another control-server / nodejs UDP) "
                    "or start with --udp-port <free-port>."
                ) from exc
            raise RuntimeError(f"Failed to bind MAVLink UDP {host}:{port}: {exc}") from exc

        self._transport = transport
        self._protocol = protocol
        logger.info("MAVLink UDP listening on %s:%s", host, port)

    async def stop(self) -> None:
        if self._transport is not None:
            self._transport.close()
            self._transport = None
            self._protocol = None

    def vehicle_addr(self) -> tuple[str, int]:
        snap = self.shared.snapshot()
        if snap.vehicle_addr is not None:
            return snap.vehicle_addr
        return (self.settings.vehicle_host, self.settings.vehicle_port)

    def _send_msg(self, msg: mavlink2.MAVLink_message) -> None:
        if self._transport is None:
            raise RuntimeError("UDP transport is not started")
        buf = msg.pack(self._mav)
        addr = self.vehicle_addr()
        self._transport.sendto(buf, addr)
        logger.debug("sent %s -> %s:%s (%d bytes)", msg.get_type(), addr[0], addr[1], len(buf))

    def request_parameter_list(self) -> None:
        msg = self._mav.param_request_list_encode(
            self.settings.vehicle_system_id,
            self.settings.vehicle_component_id,
        )
        self._send_msg(msg)

    def request_parameter_read(self, name: str | None = None, index: int = -1) -> None:
        param_id = (name or "").encode("ascii", errors="ignore")[:16]
        msg = self._mav.param_request_read_encode(
            self.settings.vehicle_system_id,
            self.settings.vehicle_component_id,
            param_id,
            index,
        )
        self._send_msg(msg)

    def set_parameter(self, name: str, value: float) -> None:
        param_id = name.encode("ascii", errors="ignore")[:16]
        msg = self._mav.param_set_encode(
            self.settings.vehicle_system_id,
            self.settings.vehicle_component_id,
            param_id,
            float(value),
            mavlink2.MAV_PARAM_TYPE_REAL32,
        )
        self._send_msg(msg)

    def send_preflight_storage(self, action: int) -> None:
        """
        MAV_CMD_PREFLIGHT_STORAGE
        action: 0=load, 1=save, 2=erase/reset (matches controller)
        """
        msg = self._mav.command_long_encode(
            self.settings.vehicle_system_id,
            self.settings.vehicle_component_id,
            mavlink2.MAV_CMD_PREFLIGHT_STORAGE,
            0,
            float(action),
            0,
            0,
            0,
            0,
            0,
            0,
        )
        self._send_msg(msg)

    def send_heartbeat(self) -> None:
        msg = self._mav.heartbeat_encode(
            mavlink2.MAV_TYPE_GCS,
            mavlink2.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavlink2.MAV_STATE_ACTIVE,
        )
        self._send_msg(msg)

    async def _notify(self) -> None:
        payload = self.shared.snapshot().to_public_dict()
        for listener in list(self._listeners):
            result = listener(payload)
            if asyncio.iscoroutine(result):
                await result

    def handle_bytes(self, data: bytes, addr: tuple[str, int]) -> None:
        for byte in data:
            try:
                msg = self._mav.parse_char(bytes([byte]))
            except Exception:
                logger.exception("MAVLink parse error")
                continue
            if msg is None:
                continue
            self._handle_message(msg, addr)

        # Schedule UI notify without blocking the datagram callback heavily.
        try:
            loop = asyncio.get_running_loop()
            loop.create_task(self._notify())
        except RuntimeError:
            pass

    def _handle_message(self, msg: mavlink2.MAVLink_message, addr: tuple[str, int]) -> None:
        msg_type = msg.get_type()

        def mutate(state) -> None:
            state.vehicle_addr = addr
            state.message_counts[msg_type] = state.message_counts.get(msg_type, 0) + 1

            if msg_type == "HEARTBEAT":
                state.heartbeat = HeartbeatSnapshot(
                    type=int(msg.type),
                    autopilot=int(msg.autopilot),
                    base_mode=int(msg.base_mode),
                    custom_mode=int(msg.custom_mode),
                    system_status=int(msg.system_status),
                    mavlink_version=int(msg.mavlink_version),
                )
            elif msg_type == "ATTITUDE":
                state.attitude = AttitudeSnapshot(
                    time_boot_ms=int(msg.time_boot_ms),
                    roll_rad=float(msg.roll),
                    pitch_rad=float(msg.pitch),
                    yaw_rad=float(msg.yaw),
                    rollspeed=float(msg.rollspeed),
                    pitchspeed=float(msg.pitchspeed),
                    yawspeed=float(msg.yawspeed),
                )
            elif msg_type == "ATTITUDE_TARGET":
                q = [
                    float(msg.q[0]),
                    float(msg.q[1]),
                    float(msg.q[2]),
                    float(msg.q[3]),
                ]
                roll_rad, pitch_rad, _ = quat_to_euler_rad(q)
                state.attitude_target = AttitudeTargetSnapshot(
                    time_boot_ms=int(msg.time_boot_ms),
                    roll_rad=roll_rad,
                    pitch_rad=pitch_rad,
                    thrust=float(msg.thrust),
                    received_mono=time.monotonic(),
                )
            elif msg_type == "RC_CHANNELS":
                channels = [
                    int(getattr(msg, f"chan{i}_raw")) for i in range(1, 19)
                ]
                state.rc_channels = RcChannelsSnapshot(
                    time_boot_ms=int(msg.time_boot_ms),
                    chancount=int(msg.chancount),
                    channels=channels,
                    rssi=int(msg.rssi),
                )
            elif msg_type == "SCALED_PRESSURE":
                temperature_c = float(msg.temperature) / 100.0
                prev = state.pressure
                state.pressure = PressureSnapshot(
                    time_boot_ms=int(msg.time_boot_ms),
                    press_abs_hpa=float(msg.press_abs),
                    press_diff_hpa=float(msg.press_diff),
                    temperature_c=temperature_c,
                    relative_altitude_m=prev.relative_altitude_m if prev else None,
                    vertical_speed_m_s=prev.vertical_speed_m_s if prev else None,
                )
            elif msg_type == "NAMED_VALUE_FLOAT":
                name = _decode_mav_string(msg.name)
                state.named_floats[name] = float(msg.value)
                if name == "AUTO_MODE":
                    state.auto_control.auto_mode = int(float(msg.value))
                    state.auto_control.diagnostics_mono = time.monotonic()
                elif name == "AUTO_PHASE":
                    state.auto_control.auto_phase = int(float(msg.value))
                    state.auto_control.diagnostics_mono = time.monotonic()
                elif name == "YAW_PROG":
                    state.auto_control.yaw_prog_deg = float(msg.value)
                    state.auto_control.diagnostics_mono = time.monotonic()
                elif name == "PRES_TGT":
                    state.auto_control.pres_tgt_hpa = float(msg.value)
                    state.auto_control.diagnostics_mono = time.monotonic()
                if state.pressure is None:
                    state.pressure = PressureSnapshot()
                if name in {"ALTITUDE", "ALTITUD", "REL_ALT"}:
                    state.pressure.relative_altitude_m = float(msg.value)
                elif name in {"VSPD", "VSPEED"}:
                    state.pressure.vertical_speed_m_s = float(msg.value)
            elif msg_type == "PARAM_VALUE":
                name = _decode_mav_string(msg.param_id)
                state.parameters[name] = ParameterValue(
                    name=name,
                    value=float(msg.param_value),
                    param_type=int(msg.param_type),
                    index=int(msg.param_index),
                    count=int(msg.param_count),
                )
            elif msg_type == "COMMAND_ACK":
                state.last_command_ack = {
                    "command": int(msg.command),
                    "result": int(msg.result),
                    "progress": int(getattr(msg, "progress", 0)),
                }

        self.shared.update(mutate)


class _MavlinkProtocol(asyncio.DatagramProtocol):
    def __init__(self, bridge: MavlinkBridge) -> None:
        self.bridge = bridge

    def datagram_received(self, data: bytes, addr: tuple[str, int]) -> None:
        self.bridge.handle_bytes(data, addr)

    def error_received(self, exc: Exception) -> None:
        logger.error("UDP error: %s", exc)


def _decode_mav_string(value: Any) -> str:
    if isinstance(value, bytes):
        return value.split(b"\x00", 1)[0].decode("ascii", errors="ignore")
    text = str(value)
    return text.split("\x00", 1)[0]
