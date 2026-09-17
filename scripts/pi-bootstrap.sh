#!/usr/bin/env bash
# Run on the Pi. Installs only the Python project; never touches CAN or motors.
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ "$(uname -s)" != Linux || "$(uname -m)" != aarch64 ]]; then
  echo 'This installer expects a 64-bit Linux Pi (aarch64).' >&2
  exit 1
fi
python3 -c 'import sys; assert sys.version_info >= (3, 11), "Python 3.11+ is required"'
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e '.[cameras]'
.venv/bin/python -m pip freeze > .venv/installed-packages.txt
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/piper-agent doctor
echo 'Base install complete. Vendor arm/camera SDKs are separate; see docs/pi-setup.md.'
