#!/usr/bin/env bash
# Start the camera-only LAN viewer on the Pi. It never connects to the arm.
set -euo pipefail
cd "$(dirname "$0")/.."
exec .venv/bin/piper-agent camera-web --config config/local.toml --bind 0.0.0.0 --port 8090
