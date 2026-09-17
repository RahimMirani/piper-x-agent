# Raspberry Pi setup

Run these commands on the Pi, not the Mac. No equipment needs to be connected for
the mock installation. Use a 64-bit Linux image with Python 3.11 or later. The
exact installed OS and legacy Orbbec wheel compatibility still need confirmation.

## 1. Base software

On a Debian/Ubuntu-based image:

```bash
sudo apt update
sudo apt install -y git python3 python3-venv python3-dev build-essential pkg-config libusb-1.0-0-dev
git clone https://github.com/RahimMirani/piper-x-agent.git
cd piper-x-agent
bash scripts/pi-bootstrap.sh
```

For a private repository, authenticate Git on the Pi first. Never put tokens in
the clone URL or commit them. The bootstrap creates `.venv` on the Pi and installs
MCP and image dependencies. It does not install vendor SDKs, configure CAN, change
USB permissions, or enable the arm. It records resolved Python versions in
`.venv/installed-packages.txt` for later diagnosis; the project currently uses
bounded dependency ranges rather than a fully locked environment.

```bash
.venv/bin/piper-agent doctor
.venv/bin/piper-agent snapshot --config config/mock.toml
.venv/bin/python -m unittest discover -s tests -v
```

Expected: `mode: mock`, two PNG patterns under `runs/<session>/`, and passing
tests including the MCP stdio test. `pyAgxArm` and `pyorbbecsdk` may be absent in
`doctor` at this stage. No GPU, model download, or inference server is needed.

## 2. Connect Codex before adding hardware

Use the [Codex guide](codex.md) with `config/mock.toml`. Ask for status and an
observation. Confirm both images arrive in Codex and the mode is explicitly mock.

## 3. Install hardware SDKs when ready

### Piper X

Use the official [pyAgxArm SDK](https://github.com/agilexrobotics/pyAgxArm), with a
revision verified for the installed firmware. Do not copy the old project's
teleoperation startup: it automatically enables and homes the arm.

Once the appropriate revision has been selected, install it into this project's
environment (replace the placeholder with an actual reviewed commit SHA):

```bash
.venv/bin/python -m pip install 'pyAgxArm @ git+https://github.com/agilexrobotics/pyAgxArm.git@<verified-commit-sha>'
```

The SDK revision is deliberately not guessed or installed by bootstrap. The old
project used `ArmModel.PIPER_X`, the `v188` profile and SocketCAN at 1 Mbps. Verify
these details on this rig. Configure the correct CAN interface only after the
adapter is identified; for a confirmed SocketCAN `can0` adapter:

```bash
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
cp config/pi.example.toml config/local.toml
.venv/bin/piper-agent probe-arm --config config/local.toml
```

The command only connects and reads joint angles; it requires SDK feedback
timestamps to advance. A successful read does not verify calibration or permit
movement. No fault clearing or firmware flashing occurs.

### Orbbec DaBai DC1 cameras

The previous working setup used the **legacy pyorbbecsdk v1 wrapper against
OrbbecSDK 1.10.35**. The correct Pi ARM64 build and Python ABI are not yet verified.
Do not blindly install the newest Orbbec wrapper or reuse an x86/macOS wheel.

Use a verified ARM64/Python-compatible v1 wheel if available; otherwise build the
legacy wrapper following its official instructions. Relevant upstream references:

- [Python wrapper releases](https://github.com/orbbec/pyorbbecsdk/releases)
- [Orbbec SDK](https://github.com/orbbec/OrbbecSDK)
- [SDK family/device support](https://orbbec.github.io/pyorbbecsdk/source/1_overview/Introduction.html)

Install the verified wheel into `.venv`, or expose the verified legacy build's
module/library paths to the SSH process. Install the vendor's Linux USB access
rules. Do not solve permissions by running the MCP server as root.

```bash
.venv/bin/piper-agent probe-cameras
.venv/bin/piper-agent snapshot-cameras --config config/local.toml
```

`probe-cameras` enumerates name, serial and firmware. `snapshot-cameras` captures
the two configured cameras **without connecting to the arm**. Cover each lens in
turn to verify the role labels. Review USB bandwidth, cable quality and power if
both cameras work individually but fail together. The capture adapter currently
expects the DC1's MJPG color profile and sends RGB JPEGs only.

## 4. Combined read-only session

After independently checking both camera roles and arm feedback:

```bash
.venv/bin/piper-agent snapshot --config config/local.toml
```

Then switch the Codex SSH command to `config/local.toml` and restart its MCP
connection. `robot_status` should show `hardware_readonly`; only three read tools
are exposed. A server instance holds the hardware process lock until it exits.
Close that connection before launching another probe/snapshot process.

Physical motion remains unimplemented. Complete the [commissioning milestones](roadmap.md)
before implementing or testing live motion.

## Updates

Close the MCP connection, then update the Pi checkout:

```bash
git pull --ff-only
.venv/bin/python -m pip install -e '.[cameras]'
.venv/bin/python -m unittest discover -s tests -v
```

Reconnect after the checks pass. The ignored `config/local.toml` remains local.
