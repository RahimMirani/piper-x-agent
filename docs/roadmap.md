# Bring-up roadmap

## Implemented for software preparation

- Python package with lazy hardware imports and explicit mock/read-only modes.
- Codex MCP server over SSH stdio; no inbound public HTTP service.
- Deterministic mock commands and two labeled diagnostic images.
- Serial-selected DC1 RGB capture with fresh-frame checks.
- Piper X joint telemetry requiring advancing SDK timestamps.
- Session event/image logging and a cooperating-process hardware lock.
- Core tests and an MCP protocol test; GitHub Actions configuration.

Implemented is not the same as hardware-validated. The Pi installation, full
software tests, and short camera-only RGB/MCP sessions have now been checked;
see [the camera commissioning record](pi-camera-check.md). Arm telemetry,
calibration, depth and physical movement remain unverified.

## Verify on the Pi before connecting equipment

1. Record OS, architecture, Python version and available storage.
2. Install project dependencies and pass the full test suite, including MCP.
3. Connect Codex over SSH and retrieve both mock images.
4. Record the resolved dependencies and measured end-to-end observation latency.

## Verify hardware independently

1. Select compatible, recorded SDK revisions/builds.
2. Confirm camera serial roles and stable simultaneous RGB capture.
3. Confirm CAN adapter, arm identity/firmware and fresh joint feedback without enabling.
4. Retrieve a combined real observation in Codex.

## Build live control after those checks

Before a live motion adapter is enabled, establish joint signs/zeros, gripper
units and stroke, tool geometry, mounting, table/workspace bounds and camera
calibration. Implement collision-aware trajectories, bounded speed/acceleration,
fresh observation checks, convergence checks and local stop/fault handling.
Verify timeout/disconnection behavior on the actual controller. A joint step
limit alone is not collision avoidance and a software stop is not an E-stop.

Start physical tests with a supervised small joint movement in clear space,
then a gripper action, then calibrated Cartesian movements. Do not treat the
mock's +/- pi range or 0.07 m aperture as validated hardware limits.

Finally attempt one large block into a bowl, logging observations and results.
No training dataset is required for the initial tool-driven experiment, but
successful manipulation is not guaranteed by a working MCP connection.
