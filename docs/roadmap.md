# Bring-up roadmap

## Implemented for software preparation

- Python package with lazy hardware imports and explicit mock/read-only modes.
- Codex MCP server over SSH stdio; no inbound public HTTP service.
- Deterministic mock commands and two labeled diagnostic images.
- Serial-selected DC1 RGB capture with fresh-frame checks.
- Piper X joint telemetry requiring advancing SDK timestamps.
- Session event/image logging and a cooperating-process hardware lock.
- Core tests and an MCP protocol test; GitHub Actions configuration.

- Supervised CLI-only motion and gripper smoke tests behind `--confirm-motion-test`.
- A `hardware_live` adapter with absolute, bounded, feedback-verified commands,
  exposed over MCP only inside an operator's time-boxed arming window.
- Trial launcher and grading sheet for comparing agents on the same arm.

Implemented is not the same as hardware-validated. The Pi installation, full
software tests, and short camera-only RGB/MCP sessions have been checked; see
[the camera commissioning record](pi-camera-check.md). Arm telemetry, a bounded
single-joint move, a lateral sweep and a gripper open/close have now also been
measured on the physical arm; see [the arm commissioning record](pi-motion-check.md).
Calibration, depth, joint signs/zeros for joints 2-6, grasping and any
collision-aware motion remain unverified. The live adapter's bounds are unit
tested, but no joint other than joint 1 has been commanded on hardware and
`move_to_pose` has never run on the physical arm.

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
then a gripper action, then calibrated Cartesian movements. The first two are
done and recorded; calibrated Cartesian movement is not. Do not treat the mock's
+/- pi range or 0.07 m aperture as validated hardware limits, and note that the
SDK silently clamps out-of-range joint targets rather than rejecting them.

Finally attempt one large block into a bowl, logging observations and results.
No training dataset is required for the initial tool-driven experiment, but
successful manipulation is not guaranteed by a working MCP connection.
