# piper-x-agent

Codex tools for a Piper X arm and two Orbbec DaBai DC1 cameras connected to a
Raspberry Pi 5. Codex runs on the Mac (or Pi); the controller runs on the Pi.
The model runs in the cloud. No model API key is used by this project.

**Current stage: hardware bring-up.** Mock operation, serial-selected RGB camera
capture, and read-only arm telemetry are implemented. Supervised single-joint,
lateral and gripper smoke tests have now been run on the physical arm; see the
[arm commissioning record](docs/pi-motion-check.md). Calibration, depth
processing, and collision-aware planning are not implemented.

Physical motion is available two ways. Supervised smoke tests run from the CLI
behind `--confirm-motion-test`. Bounded model-driven motion runs in
`hardware_live` mode, which has to be selected explicitly by pointing the server
at a live configuration. In `mock` and `hardware_readonly` modes no MCP tool
moves the arm.

```text
Mac: Codex with Astra          OpenAI cloud: model inference
        |                            ^
        +----------------------------+
        |
        | MCP over SSH (standard input/output)
        v
Pi 5: piper-x-agent -> pyAgxArm -> USB-CAN -> Piper X
                   -> legacy Orbbec SDK -> scene + wrist cameras
```

Running tools on the Pi keeps hardware communication local. Image transfer,
network latency, and model reasoning still take time. This is a discrete
observe/act workflow, not a real-time motor controller.

## Start on the Pi, without hardware

Requires 64-bit Linux and Python 3.11+. The Mac does not need a Python environment.

```bash
git clone https://github.com/RahimMirani/piper-x-agent.git
cd piper-x-agent
bash scripts/pi-bootstrap.sh
.venv/bin/piper-agent snapshot --config config/mock.toml
```

The snapshot writes two **diagnostic patterns**, metadata, and a JSONL event log
under `runs/`. Mock mode has no physics, object detection, or real camera feed;
its purpose is to verify the interface before connecting equipment.

Follow [Pi setup](docs/pi-setup.md), then [connect Codex](docs/codex.md).
The [hardware notes](docs/hardware.md) record the camera identities found in the
earlier `piper-x-arm` project. They must be confirmed against the physical rig.
The [Pi camera commissioning record](docs/pi-camera-check.md) documents the
verified installation, camera images and MCP checks, including one transient timeout.
The [arm commissioning record](docs/pi-motion-check.md) documents the CAN bring-up,
the three feedback defects found before any result was trusted, and the measured
motion and gripper checks.

## Tools

| Tool | Mock mode | Hardware read-only mode |
| --- | --- | --- |
| `robot_status` | Mode and capabilities | Mode and capabilities; no connection needed |
| `read_arm_state` | Mock joint angles | Fresh joint feedback in radians |
| `observe` | Two diagnostic images + mock state | Two fresh RGB images + joint state |
| `observe_cameras` | Two diagnostic images only | Both RGB feeds without connecting to the arm |
| `simulate_joint_move` | Bounded mock state update | Not exposed |
| `simulate_gripper` | Mock aperture update | Not exposed |
| `simulate_stop` | Latched mock stop | Not exposed; cannot stop a real arm |

In `hardware_live` mode five more tools appear:
`move_joints` and `move_to_pose` take **absolute** targets and move at most one
bounded step per call, `set_gripper` sets an absolute jaw width, `stop` requests
a damped software stop, and `done` records the model's own end-of-episode claim.
Out-of-range targets are refused rather than clamped, and every call returns the
**measured** result rather than the commanded one. There is no collision
checking: the step limits keep one call small, they do not know where the table
is.

### Supervised motion tests (CLI only, not MCP tools)

```bash
.venv/bin/piper-agent motion-test  --config config/local.toml --joint 1 --delta 0.2 --confirm-motion-test
.venv/bin/piper-agent lateral-test --config config/local.toml --delta 0.2 --confirm-motion-test
.venv/bin/piper-agent gripper-test --config config/local.toml --confirm-motion-test
```

Each runs at 10% speed, offsets at most 0.2 rad, verifies measured displacement
against advancing SDK feedback, and returns to the starting pose. They leave the
arm enabled and holding, because disabling a raised arm lets it drop. Run them
only with the workspace clear and an operator at the rig. There is no collision
checking; `electronic_emergency_stop()` is not a hardware E-stop.

### Evaluating models on the arm

`hardware_live` exists to compare agents driving the same arm through the same
tools. `scripts/run-trials.sh` runs a batch of trials, each in a fresh session in
an empty directory, restricted to the arm's MCP tools; `scripts/grade-trials.py`
builds a grading sheet a human fills in from video. The method, and the ways it
can mislead, are in [the evaluation protocol](docs/eval-protocol.md).

Hardware SDKs are imported only when their tools are used. Startup never enables,
homes, re-zeros, changes modes, clears faults, or modifies the gripper. Shutdown
disconnects without disabling motors. A per-user process lock prevents competing
instances of this project's hardware server; it does not block unrelated SDK
programs or another OS user.

## Development and verification

On the Pi after installation:

```bash
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/piper-agent doctor
```

Core tests also run without installing dependencies using an existing Python
3.11+ interpreter. The MCP transport test skips if `mcp` is missing; GitHub Actions
installs the package and runs it on Python 3.11 and 3.12. CI checks software, not
ARM64 vendor binaries or physical hardware. Check the actual Actions result;
adding a workflow is not evidence it passed.

`runs/`, `.venv/`, and `config/local.toml` stay out of Git. Logs and images can
grow without a retention limit; periodically archive/remove old sessions on the
Pi. Observation images are sent to the model when Codex calls `observe`.

Next milestones and commissioning evidence are in [the roadmap](docs/roadmap.md).
