#!/usr/bin/env bash
# SSH stdio entrypoint. No activation scripts, banners or background service.
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ $# -ne 1 ]]; then
  echo 'Usage: bash scripts/pi-mcp.sh config/mock.toml|config/local.toml' >&2
  exit 2
fi
exec .venv/bin/piper-agent serve --config "$1"
