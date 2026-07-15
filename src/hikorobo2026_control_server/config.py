from __future__ import annotations

from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
    )

    # UDP listen port on the PC (matches controller passwd.h HOST_PORT)
    udp_listen_host: str = "0.0.0.0"
    udp_listen_port: int = 5000

    # Fallback destination if no telemetry has been received yet
    # (matches controller LOCAL_IP / LOCAL_PORT)
    vehicle_host: str = "192.168.0.15"
    vehicle_port: int = 1234

    # GCS identity (CURSOR 03-mavlink.md)
    gcs_system_id: int = 255
    gcs_component_id: int = 190  # MAV_COMP_ID_MISSIONPLANNER

    # Expected vehicle identity
    vehicle_system_id: int = 1
    vehicle_component_id: int = 25  # MAV_COMP_ID_USER1

    # HTTP / WebSocket UI
    http_host: str = "0.0.0.0"
    http_port: int = 3000

    csv_log_dir: str = "logs"


settings = Settings()
