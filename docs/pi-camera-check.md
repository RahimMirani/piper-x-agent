# Pi camera commissioning — 2026-09-19

Camera-only setup completed on the user's Pi. No CAN configuration, arm SDK
installation, arm connection, enabling, homing, or motor commands were performed.

## Verified environment

- Raspberry Pi running Debian 13.5 (trixie), Linux aarch64.
- Project installed in `/home/rahim/piper-x-agent/.venv` using Python 3.13.13.
- MCP 1.30.0, NumPy 2.5.3, OpenCV headless 4.14.0.94.
- Reused existing legacy camera wrapper checkout `~/trc/pyorbbecsdk`, commit
  `ee32b47`, with an existing CPython 3.13 ARM64 binary.
- Loaded SDK reports **OrbbecSDK 1.10.16**, not the 1.10.35 build mentioned in
  historical notes. This existing build successfully captured both cameras.
- Both cameras report DaBai DC1, firmware RD1001; scene serial `CC1N16200TF`,
  wrist serial `CC1N16200F9`. Actual images are consistent with the assigned roles.
- RGB profile: MJPG, 640 × 480, 30 FPS configured on each camera. This is the
  requested device profile, not a measured sustained delivery rate.

The project environment has an ignored `.pth` file named `orbbec-local.pth` in
its site-packages directory pointing to `/home/rahim/trc/pyorbbecsdk/install/lib`.
This exposes the existing camera module without changing global Python or
copying vendor binaries into Git. Recreating `.venv` requires restoring this
entry. With the same verified SDK path on this Pi:

```bash
.venv/bin/python - <<'PY'
from pathlib import Path
import sysconfig
sdk = Path.home() / 'trc/pyorbbecsdk/install/lib'
if not sdk.is_dir():
    raise SystemExit('Verified SDK build not found')
Path(sysconfig.get_paths()['purelib'], 'orbbec-local.pth').write_text(str(sdk) + '\n')
PY
```

## Checks performed

- All 10 project tests passed on the Pi, including the actual MCP stdio test.
- Enumerated both cameras and saved/visually inspected both RGB images.
- Captured 60 consecutive image pairs (120 images), requiring advancing device
  timestamps. Mean capture-call duration: 57 ms, including first initialization.
  Maximum frame age on return: 59 ms. Maximum pair receipt skew: 55 ms.
- Confirmed the camera-only runtime never instantiated an arm backend.
- Tested actual RGB images through the MCP `observe_cameras` tool: an initial
  attempt timed out, a retry returned three pairs successfully, and three further
  fresh server starts each returned ten successful pairs. Initial timeout cause
  is unresolved; no automatic fallback or stale-frame bypass was added.
- All MCP rechecks reported `hardware_connected: false`.

The SDK emits legacy warnings including unsupported frame sync and USB core
type queries, even on successful runs. These checks validate short RGB capture
sessions, not long-duration reliability, depth, calibrated geometry or manipulation.
Capture-call duration excludes cloud inference, image upload, and Mac-to-Pi SSH.

## Saved evidence on the Pi

- First image pair: `runs/20260920T031942Z-4ad170a4/`.
- 60-pair stability run and `camera-check.json`:
  `runs/20260920T032044Z-b7612d7b/`.
- MCP diagnostic logs: `/tmp/piper-camera-mcp.log`,
  `/tmp/piper-camera-mcp-retry.log`, `/tmp/piper-mcp-repeat.log`.

The run names use UTC, hence the next-day date. Images, SDK `Log/` output, local
configuration and credentials are excluded from Git.

The scene view shows the table and room; the wrist view shows the table with the
gripper in the foreground. Confirm the final target area and camera placement
before calibration. Depth and physical motion have not been tested. The MCP
hardware path is verified locally on the Pi; persistent registration in the Mac
Codex client still needs SSH key authentication and the documented MCP entry.
