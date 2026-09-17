# Working on piper-x-agent

- Current target is Codex and a Raspberry Pi 5. Do not install a Python environment
  on the Mac unless the user asks; run dependency-backed tests on Pi/CI. Core
  unittest tests can use an existing Python 3.11+ interpreter without installation.
- User authorizes small coherent commits and pushes to `origin main`. Keep doing
  both after relevant checks; do not repeatedly ask for conversational approval.
- Never include credentials, `.venv`, local hardware config, recordings or run logs
  in commits. Commit source, examples, tests and documentation.
- The v0.1 hardware adapter is telemetry-only. Do not enable/home/zero/flash an arm
  or clear faults as an incidental setup action. New live control requires explicit
  user direction and verified hardware/calibration, not just a configuration flag.
- Keep mock observations unmistakable; mock tests do not establish physical safety
  or grasp success. No automatic fallback to mock after a hardware error.
- Keep SDK imports lazy so mocks work without vendor packages. Preserve serial-based
  camera selection and reject stale observations rather than relabeling cached data.
- Reserve stdout for MCP in server mode, including native-library output. Logs go
  to stderr or ignored run directories. No network listener is needed for SSH stdio.
- Report tests actually run, skipped tests, and hardware checks still outstanding.
