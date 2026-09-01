# インストール

`hikorobo2026_control_server` のセットアップ手順です。

## 必要条件

- Python 3.12 以上
- [uv](https://docs.astral.sh/uv/)（依存関係のインストールと実行に使用）

## 1. Python 3.12+ のインストール

未導入の場合は、[python.org](https://www.python.org/downloads/) から 3.12 以上を入れてください。

バージョン確認:

```bash
python --version
```

Windows で `python` が見つからない場合は `py -3.12 --version` を試してください。

## 2. uv のインストール

### Windows (PowerShell)

```powershell
irm https://astral.sh/uv/install.ps1 | iex
```

### macOS / Linux

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### その他

- Homebrew: `brew install uv`
- pip: `pip install uv`

インストール後、新しいターミナルを開き、次で確認します。

```bash
uv --version
```

## 3. リポジトリの取得

```bash
git clone <このリポジトリの URL>
cd hikorobo2026_control_server
```

すでにクローン済みなら、プロジェクトのルートへ移動してください。

## 4. 依存関係のインストール

プロジェクトルートで:

```bash
uv sync
```

`uv` が `.python-version`（3.12）に合わせて仮想環境を作り、`pyproject.toml` の依存パッケージを入れます。

## 5. 設定ファイル

```bash
cp .env.example .env
```

Windows (PowerShell) では:

```powershell
Copy-Item .env.example .env
```

必要に応じて `.env` を編集します。主な項目:

| 変数 | 既定値 | 説明 |
| --- | --- | --- |
| `UDP_LISTEN_PORT` | `5000` | ESP32 テレメトリの受信ポート（controller の `HOST_PORT`） |
| `VEHICLE_HOST` | `192.168.0.15` | 初回受信前のコマンド送信先 IP |
| `VEHICLE_PORT` | `1234` | 初回受信前のコマンド送信先ポート |
| `HTTP_PORT` | `3000` | Web UI のポート |

詳細は [README.md](README.md) の「設定」を参照してください。

## 起動確認

ESP32 より先にサーバを起動してください。

```bash
uv run hikorobo2026-control-server
```

または:

```bash
uv run python -m hikorobo2026_control_server
```

ブラウザで [http://127.0.0.1:3000](http://127.0.0.1:3000) を開き、Web UI が表示されれば完了です。

> `http://0.0.0.0:3000` はブラウザでは使えません。`127.0.0.1` か `localhost` を使ってください。
