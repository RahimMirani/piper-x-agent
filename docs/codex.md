# Connect Codex on the Mac to the Pi

The Pi runs the MCP server and hardware SDKs. Codex signs in on the Mac and calls
those tools over SSH. The Python package makes no OpenAI API requests and does not
need an API key. Your Codex account's model availability and usage limits apply.

## SSH prerequisite

Set up SSH key authentication and verify the Pi's host key normally. Example
alias in your Mac's `~/.ssh/config` (replace the hostname and username):

```sshconfig
Host pi
    HostName YOUR_PI_HOSTNAME
    User YOUR_PI_USER
    IdentityFile ~/.ssh/id_ed25519
    ServerAliveInterval 15
    ServerAliveCountMax 3
```

Verify an ordinary SSH login first, then:

```bash
ssh -T -o BatchMode=yes pi 'uname -m'
```

Keep shell startup files quiet for noninteractive SSH. MCP uses stdout for JSON
messages; banners or `echo` output from remote shell startup can break it.

## Register the mock server

After the Pi installation, run this on the Mac. Replace `/home/YOUR_PI_USER` with
the actual absolute path to the Pi checkout:

```bash
codex mcp add piper-x -- ssh -T -o BatchMode=yes pi 'bash /home/YOUR_PI_USER/piper-x-agent/scripts/pi-mcp.sh config/mock.toml'
```

Or configure the equivalent MCP entry in the desktop app. Example TOML:

```toml
[mcp_servers.piper-x]
command = "ssh"
args = ["-T", "-o", "BatchMode=yes", "pi", "bash /home/YOUR_PI_USER/piper-x-agent/scripts/pi-mcp.sh config/mock.toml"]
startup_timeout_sec = 30
tool_timeout_sec = 30
```

Configuration is documented here rather than auto-installed: the Pi hostname,
username and checkout path are not known yet. Local Codex clients share MCP
configuration on the same host. See [official MCP documentation](https://learn.chatgpt.com/docs/extend/mcp).

Restart/reconnect the server, select Astra in Codex, and ask:

> Use the piper-x tools. Check robot_status, then observe. Confirm the mode and
> identify the scene and wrist diagnostic images. Make one mock joint move of
> 0.05 radians on joint 1 with the other five joints unchanged, then read the
> state. Do not use shell commands to access hardware.

The mock updates state instantly and has no collision or dynamics model. The
images do not change to depict motion. A successful test establishes the tool
connection only.

## Later: read-only hardware

Finish the Pi hardware checks, then replace `config/mock.toml` in the MCP command
with `config/local.toml`. Restart the MCP connection and verify `robot_status`.
Hardware mode exposes `robot_status`, `read_arm_state`, `observe`, and
`observe_cameras`. Use `observe_cameras` during camera-only setup; it does not
initialize the arm SDK or connect to CAN.

Do not run another hardware server/probe while this connection holds its lock.
The server does not auto-start at boot. Closing Codex/SSH ends the session; it
does not disable or stop an arm controlled by another program.

## Timing expectations

The Pi consumes camera frames continuously and returns a fresh pair on each
`observe` call, resized to at most 960 pixels on the longest edge. The views are
not hardware-synchronized. Metadata includes receipt timestamps, frame age,
capture skew and tool elapsed time. These do not measure cloud inference latency
or establish camera-to-robot calibration.

Your images still travel to the cloud model through Codex. Reasoning and network
delays remain whether Codex runs on the Mac or directly on the Pi. Future live
motion must execute bounded actions with local feedback and independently tested
stopping behavior; the model cannot provide a guaranteed motor control rate.
