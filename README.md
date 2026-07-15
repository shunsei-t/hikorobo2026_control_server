# hikorobo2026 Control Server

`hikorobo2026_controller` 向けの MAVLink 2 GCS（地上局）です。  
ESP32 からの UDP テレメトリを受信し、パラメータ読み書き・NVS 保存・簡易 Web UI を提供します。

## 必要条件

- Python 3.12+
- [uv](https://docs.astral.sh/uv/)

## セットアップ

```bash
cd hikorobo2026_control_server
uv sync
cp .env.example .env   # 必要なら編集
```

## 起動

ESP32 より先にサーバを起動してください。

```bash
uv run hikorobo2026-control-server
```

または:

```bash
uv run python -m hikorobo2026_control_server
```

- Web UI: http://127.0.0.1:3000
- UDP 受信: `0.0.0.0:5000`（controller の `HOST_PORT`）
- コマンド送信先: 受信元アドレス、なければ `.env` の `VEHICLE_HOST:VEHICLE_PORT`（既定 `192.168.0.15:1234`）

> ブラウザでは `http://0.0.0.0:3000` は使えません。`http://127.0.0.1:3000` か `http://localhost:3000` を開いてください。

## controller との対応

| controller (`passwd.h`) | server |
| --- | --- |
| `HOST_PORT` (5000) | UDP listen port |
| `LOCAL_IP` / `LOCAL_PORT` | vehicle target |
| System ID 1 / `MAV_COMP_ID_USER1` | vehicle |
| System ID 255 / Mission Planner component | GCS |

### 受信メッセージ

- `HEARTBEAT`（`custom_mode` = flight state）
- `ATTITUDE`
- `RC_CHANNELS`
- `SCALED_PRESSURE`
- `NAMED_VALUE_FLOAT`（`ALTITUDE`, `VSPD`）
- `PARAM_VALUE`
- `COMMAND_ACK`

### 送信メッセージ

- `PARAM_REQUEST_LIST` / `PARAM_REQUEST_READ` / `PARAM_SET`
- `MAV_CMD_PREFLIGHT_STORAGE`（0=load, 1=save, 2=reset）
- GCS `HEARTBEAT`（1 Hz）

## API 概要

| Method | Path | 説明 |
| --- | --- | --- |
| GET | `/api/health` | 接続状態 |
| GET | `/api/telemetry` | 最新テレメトリ JSON |
| GET | `/api/parameters` | キャッシュ済みパラメータ |
| POST | `/api/parameters/list` | 全パラメータ要求 |
| POST | `/api/parameters/set` | `{"name":"ROLL_P","value":1.2}` |
| POST | `/api/storage` | `{"action":1}` NVS 保存 |
| WS | `/ws` | テレメトリ配信 |

## 設定

環境変数または `.env`:

```env
UDP_LISTEN_PORT=5000
VEHICLE_HOST=192.168.0.15
VEHICLE_PORT=1234
HTTP_PORT=3000
```
