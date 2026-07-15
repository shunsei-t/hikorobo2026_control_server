from __future__ import annotations

import csv
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, TextIO


class CsvTelemetryLogger:
    def __init__(self, log_dir: str | Path) -> None:
        self.log_dir = Path(log_dir)
        self._path: Path | None = None
        self._fp: TextIO | None = None
        self._writer: csv.DictWriter | None = None

    @property
    def active(self) -> bool:
        return self._fp is not None

    @property
    def path(self) -> str | None:
        return str(self._path) if self._path else None

    def start(self) -> str:
        if self.active:
            return self.path or ""

        self.log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H-%M-%S")
        self._path = self.log_dir / f"mavlink_{stamp}.csv"
        self._fp = self._path.open("w", newline="", encoding="utf-8")
        fieldnames = [
            "timestamp_iso",
            "flight_state",
            "flight_state_label",
            "roll_rad",
            "pitch_rad",
            "yaw_rad",
            "roll_deg",
            "pitch_deg",
            "yaw_deg",
            "press_abs_hpa",
            "temperature_c",
            "relative_altitude_m",
            "vertical_speed_m_s",
            "rc_1",
            "rc_2",
            "rc_3",
            "rc_4",
            "rc_5",
            "rc_6",
            "rc_7",
            "rc_8",
        ]
        self._writer = csv.DictWriter(self._fp, fieldnames=fieldnames)
        self._writer.writeheader()
        self._fp.flush()
        return str(self._path)

    def stop(self) -> None:
        if self._fp is not None:
            self._fp.close()
        self._fp = None
        self._writer = None
        self._path = None

    def write(self, telemetry: dict[str, Any]) -> None:
        if self._writer is None or self._fp is None:
            return

        attitude = telemetry.get("attitude") or {}
        heartbeat = telemetry.get("heartbeat") or {}
        pressure = telemetry.get("pressure") or {}
        rc = telemetry.get("rc_channels") or {}
        channels = rc.get("channels") or []

        row = {
            "timestamp_iso": datetime.now(timezone.utc).isoformat(),
            "flight_state": heartbeat.get("flight_state", ""),
            "flight_state_label": heartbeat.get("flight_state_label", ""),
            "roll_rad": attitude.get("roll_rad", ""),
            "pitch_rad": attitude.get("pitch_rad", ""),
            "yaw_rad": attitude.get("yaw_rad", ""),
            "roll_deg": attitude.get("roll_deg", ""),
            "pitch_deg": attitude.get("pitch_deg", ""),
            "yaw_deg": attitude.get("yaw_deg", ""),
            "press_abs_hpa": pressure.get("press_abs_hpa", ""),
            "temperature_c": pressure.get("temperature_c", ""),
            "relative_altitude_m": pressure.get("relative_altitude_m", ""),
            "vertical_speed_m_s": pressure.get("vertical_speed_m_s", ""),
            "rc_1": channels[0] if len(channels) > 0 else "",
            "rc_2": channels[1] if len(channels) > 1 else "",
            "rc_3": channels[2] if len(channels) > 2 else "",
            "rc_4": channels[3] if len(channels) > 3 else "",
            "rc_5": channels[4] if len(channels) > 4 else "",
            "rc_6": channels[5] if len(channels) > 5 else "",
            "rc_7": channels[6] if len(channels) > 6 else "",
            "rc_8": channels[7] if len(channels) > 7 else "",
        }
        self._writer.writerow(row)
        self._fp.flush()
