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
            *[f"rc_{i}" for i in range(1, 19)],
            "target_roll_deg",
            "target_pitch_deg",
            "target_thr_pwm",
            "auto_mode",
            "auto_phase",
            "yaw_prog_deg",
            "pres_tgt_hpa",
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
        att_tgt = telemetry.get("attitude_target") or {}
        auto = telemetry.get("auto_control") or {}
        named = telemetry.get("named_floats") or {}

        is_auto = heartbeat.get("flight_state") == 2
        tgt_roll = att_tgt.get("roll_deg", "") if is_auto and att_tgt.get("active") else ""
        tgt_pitch = att_tgt.get("pitch_deg", "") if is_auto and att_tgt.get("active") else ""
        tgt_thr = att_tgt.get("target_thr_pwm", "") if is_auto and att_tgt.get("active") else ""

        auto_mode = auto.get("auto_mode", "") if is_auto else ""
        auto_phase = auto.get("auto_phase", "") if is_auto else ""
        yaw_prog = (
            auto.get("yaw_prog_deg")
            if is_auto and auto.get("yaw_prog_deg") is not None
            else named.get("YAW_PROG", "")
        )
        pres_tgt = (
            auto.get("pres_tgt_hpa")
            if is_auto and auto.get("pres_tgt_hpa") is not None
            else named.get("PRES_TGT", "")
        )

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
            **{
                f"rc_{i}": channels[i - 1] if len(channels) >= i else ""
                for i in range(1, 19)
            },
            "target_roll_deg": tgt_roll,
            "target_pitch_deg": tgt_pitch,
            "target_thr_pwm": tgt_thr,
            "auto_mode": auto_mode,
            "auto_phase": auto_phase,
            "yaw_prog_deg": yaw_prog if is_auto else "",
            "pres_tgt_hpa": pres_tgt if is_auto else "",
        }
        self._writer.writerow(row)
        self._fp.flush()
