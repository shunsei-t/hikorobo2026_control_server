from __future__ import annotations

import argparse
import logging

import uvicorn

from hikorobo2026_control_server.api import create_app
from hikorobo2026_control_server.config import settings


def main() -> None:
    parser = argparse.ArgumentParser(
        description="MAVLink control server for hikorobo2026_controller",
    )
    parser.add_argument("--host", default=settings.http_host, help="HTTP bind host")
    parser.add_argument(
        "--port",
        type=int,
        default=settings.http_port,
        help="HTTP bind port",
    )
    parser.add_argument(
        "--udp-port",
        type=int,
        default=settings.udp_listen_port,
        help="UDP listen port (controller HOST_PORT)",
    )
    parser.add_argument(
        "--vehicle-host",
        default=settings.vehicle_host,
        help="Fallback ESP32 IP before first telemetry packet",
    )
    parser.add_argument(
        "--vehicle-port",
        type=int,
        default=settings.vehicle_port,
        help="Fallback ESP32 UDP port (controller LOCAL_PORT)",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
    )
    args = parser.parse_args()

    settings.http_host = args.host
    settings.http_port = args.port
    settings.udp_listen_port = args.udp_port
    settings.vehicle_host = args.vehicle_host
    settings.vehicle_port = args.vehicle_port

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
    )

    app = create_app(settings)
    uvicorn.run(app, host=args.host, port=args.port, log_level=args.log_level.lower())


if __name__ == "__main__":
    main()
