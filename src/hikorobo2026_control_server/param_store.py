from __future__ import annotations

import csv
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


_SAFE_NAME = re.compile(r"^[A-Za-z0-9._-]{1,64}$")


class ParameterFileStore:
    """Save / load MAVLink parameter snapshots as CSV (name,value)."""

    def __init__(self, directory: str | Path) -> None:
        self.directory = Path(directory)

    def ensure_dir(self) -> None:
        self.directory.mkdir(parents=True, exist_ok=True)

    def list_files(self) -> list[dict[str, Any]]:
        self.ensure_dir()
        rows: list[dict[str, Any]] = []
        for path in sorted(self.directory.glob("*.csv"), key=lambda p: p.stat().st_mtime, reverse=True):
            rows.append(
                {
                    "name": path.name,
                    "path": str(path),
                    "size": path.stat().st_size,
                    "mtime_iso": datetime.fromtimestamp(
                        path.stat().st_mtime, tz=timezone.utc
                    ).isoformat(),
                }
            )
        return rows

    def resolve(self, name: str) -> Path:
        if not _SAFE_NAME.match(name) or not name.endswith(".csv"):
            raise ValueError("invalid file name")
        path = (self.directory / name).resolve()
        if path.parent != self.directory.resolve():
            raise ValueError("path escapes store directory")
        return path

    def save(
        self,
        parameters: dict[str, Any],
        name: str | None = None,
    ) -> Path:
        self.ensure_dir()
        if name:
            if not name.endswith(".csv"):
                name = f"{name}.csv"
            path = self.resolve(name)
        else:
            stamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H-%M-%S")
            path = self.directory / f"params_{stamp}.csv"

        rows = self._parameters_to_rows(parameters)
        with path.open("w", newline="", encoding="utf-8") as fp:
            writer = csv.DictWriter(fp, fieldnames=["name", "value"])
            writer.writeheader()
            writer.writerows(rows)
        return path

    def load(self, name: str) -> list[tuple[str, float]]:
        path = self.resolve(name)
        if not path.is_file():
            raise FileNotFoundError(name)
        return self.parse_csv_text(path.read_text(encoding="utf-8"))

    def export_text(self, parameters: dict[str, Any]) -> str:
        rows = self._parameters_to_rows(parameters)
        from io import StringIO

        buf = StringIO()
        writer = csv.DictWriter(buf, fieldnames=["name", "value"])
        writer.writeheader()
        writer.writerows(rows)
        return buf.getvalue()

    @staticmethod
    def parse_csv_text(text: str) -> list[tuple[str, float]]:
        from io import StringIO

        reader = csv.DictReader(StringIO(text))
        if not reader.fieldnames:
            raise ValueError("empty CSV")
        fields = {f.strip().lower(): f for f in reader.fieldnames if f}
        name_key = fields.get("name") or fields.get("param") or fields.get("parameter")
        value_key = fields.get("value") or fields.get("val")
        if name_key is None or value_key is None:
            raise ValueError("CSV must have name,value columns")

        out: list[tuple[str, float]] = []
        for row in reader:
            raw_name = (row.get(name_key) or "").strip()
            raw_value = (row.get(value_key) or "").strip()
            if not raw_name or raw_value == "":
                continue
            name = raw_name[:16]
            try:
                value = float(raw_value)
            except ValueError as exc:
                raise ValueError(f"invalid value for {name}: {raw_value}") from exc
            out.append((name, value))
        return out

    @staticmethod
    def _parameters_to_rows(parameters: dict[str, Any]) -> list[dict[str, str]]:
        rows: list[dict[str, str]] = []
        for name in sorted(parameters.keys()):
            entry = parameters[name]
            if isinstance(entry, dict):
                value = entry.get("value")
            else:
                value = getattr(entry, "value", entry)
            rows.append({"name": name, "value": str(float(value))})
        return rows
