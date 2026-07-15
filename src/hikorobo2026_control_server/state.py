from __future__ import annotations

import math
import threading
import time
from dataclasses import asdict, dataclass, field
from typing import Any


FLIGHT_STATE_LABELS = {
    0: "INIT",
    1: "ERROR",
    2: "AUTO",
    3: "SEMIAUTO",
    4: "MANUAL",
    5: "SBUS_LOST",
}


@dataclass
class AttitudeSnapshot:
    time_boot_ms: int = 0
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0
    rollspeed: float = 0.0
    pitchspeed: float = 0.0
    yawspeed: float = 0.0

    @property
    def roll_deg(self) -> float:
        return math.degrees(self.roll_rad)

    @property
    def pitch_deg(self) -> float:
        return math.degrees(self.pitch_rad)

    @property
    def yaw_deg(self) -> float:
        return math.degrees(self.yaw_rad)


@dataclass
class RcChannelsSnapshot:
    time_boot_ms: int = 0
    chancount: int = 0
    channels: list[int] = field(default_factory=lambda: [0] * 18)
    rssi: int = 255


@dataclass
class PressureSnapshot:
    time_boot_ms: int = 0
    press_abs_hpa: float = 0.0
    press_diff_hpa: float = 0.0
    temperature_c: float = 0.0
    relative_altitude_m: float | None = None
    vertical_speed_m_s: float | None = None


@dataclass
class HeartbeatSnapshot:
    type: int = 0
    autopilot: int = 0
    base_mode: int = 0
    custom_mode: int = 0
    system_status: int = 0
    mavlink_version: int = 0

    @property
    def flight_state(self) -> int:
        return int(self.custom_mode)

    @property
    def flight_state_label(self) -> str:
        return FLIGHT_STATE_LABELS.get(self.flight_state, f"UNKNOWN({self.flight_state})")


@dataclass
class ParameterValue:
    name: str
    value: float
    param_type: int
    index: int
    count: int


@dataclass
class TelemetryState:
    last_update_mono: float = 0.0
    vehicle_addr: tuple[str, int] | None = None
    heartbeat: HeartbeatSnapshot | None = None
    attitude: AttitudeSnapshot | None = None
    rc_channels: RcChannelsSnapshot | None = None
    pressure: PressureSnapshot | None = None
    named_floats: dict[str, float] = field(default_factory=dict)
    parameters: dict[str, ParameterValue] = field(default_factory=dict)
    last_command_ack: dict[str, Any] | None = None
    message_counts: dict[str, int] = field(default_factory=dict)

    def touch(self) -> None:
        self.last_update_mono = time.monotonic()

    def age_s(self) -> float | None:
        if self.last_update_mono <= 0.0:
            return None
        return time.monotonic() - self.last_update_mono

    def to_public_dict(self) -> dict[str, Any]:
        attitude = None
        if self.attitude is not None:
            attitude = {
                **asdict(self.attitude),
                "roll_deg": self.attitude.roll_deg,
                "pitch_deg": self.attitude.pitch_deg,
                "yaw_deg": self.attitude.yaw_deg,
            }

        heartbeat = None
        if self.heartbeat is not None:
            heartbeat = {
                **asdict(self.heartbeat),
                "flight_state": self.heartbeat.flight_state,
                "flight_state_label": self.heartbeat.flight_state_label,
            }

        return {
            "connected": (self.age_s() is not None) and (self.age_s() < 2.0),
            "age_s": self.age_s(),
            "vehicle_addr": (
                {"host": self.vehicle_addr[0], "port": self.vehicle_addr[1]}
                if self.vehicle_addr
                else None
            ),
            "heartbeat": heartbeat,
            "attitude": attitude,
            "rc_channels": asdict(self.rc_channels) if self.rc_channels else None,
            "pressure": asdict(self.pressure) if self.pressure else None,
            "named_floats": dict(self.named_floats),
            "parameters": {
                name: asdict(value) for name, value in sorted(self.parameters.items())
            },
            "last_command_ack": self.last_command_ack,
            "message_counts": dict(self.message_counts),
        }


class SharedState:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self.telemetry = TelemetryState()

    def update(self, mutator) -> TelemetryState:
        with self._lock:
            mutator(self.telemetry)
            self.telemetry.touch()
            return self.snapshot()

    def snapshot(self) -> TelemetryState:
        with self._lock:
            # Shallow copy of containers used by the UI
            snap = TelemetryState(
                last_update_mono=self.telemetry.last_update_mono,
                vehicle_addr=self.telemetry.vehicle_addr,
                heartbeat=self.telemetry.heartbeat,
                attitude=self.telemetry.attitude,
                rc_channels=self.telemetry.rc_channels,
                pressure=self.telemetry.pressure,
                named_floats=dict(self.telemetry.named_floats),
                parameters=dict(self.telemetry.parameters),
                last_command_ack=self.telemetry.last_command_ack,
                message_counts=dict(self.telemetry.message_counts),
            )
            return snap
