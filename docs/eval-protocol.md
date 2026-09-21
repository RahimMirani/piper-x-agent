# Evaluating agents on the PiPER-X

How to compare Astra-in-Codex against Fable-in-Claude-Code on a real arm without
the harness quietly deciding the result.

## What this measures

**Deployed systems, not bare models.** Each trial runs the vendor's own agent
scaffold — its system prompt, retry behaviour and context handling — over MCP.
That is how the arm will really be driven, so it is a fair thing to measure, but
it is not "Astra vs Fable" in isolation. Report it as what it is.

A separate direct-API harness would isolate the models. It is not built, and it
is the thing to build if a scaffold difference turns out to explain a gap.

## The control interface

Absolute targets only, one bounded step per call:

| Tool | Contract |
| --- | --- |
| `move_joints` | Absolute six-joint pose, radians |
| `move_to_pose` | Absolute flange pose in the arm base frame, metres and radians |
| `set_gripper` | Absolute jaw width, metres |
| `stop` | Damped software stop; **not** a hardware E-stop |
| `done` | The model's own end-of-episode claim |

Absolute rather than relative, so a call retried after a transport timeout is
harmless. Out-of-range targets are **refused, not clamped** — the SDK's own
`clamp_joints` silently moves joints nobody asked about, which is how an early
motion test reported a pass while the arm sat still.

Run `move_joints` and `move_to_pose` as separate conditions. The joint-space
condition is the interesting one: it removes the IK pipeline that does quiet
work in the published Astra setup.

## Before a batch

1. Workspace clear, operator at the rig, power cutoff in reach.
2. `piper-agent arm --minutes 45` on the Pi. Motion tools are not listed at all
   outside that window, and it expires on its own.
3. Cube and cup on marked positions. Tape the table — position variance between
   trials becomes noise that swamps the difference you are looking for.
4. External camera recording. The Pi saves a frame per `observe`, which shows
   the arm at each decision point but not the motion between them.

## Running

```bash
scripts/run-trials.sh --agent claude --trials 20 --task cube-in-cup
scripts/run-trials.sh --agent codex  --trials 20 --task cube-in-cup
scripts/grade-trials.py runs-eval/<batch>
```

Claude trials pass the MCP server inline, so nothing needs registering. Codex
trials do not: `codex exec` has no equivalent flag, so register the server once
with `codex mcp add` before the first Codex batch, pointing at `config/live.toml`.
Both paths need key-based SSH, since the agent spawns the connection itself with
no terminal to answer a password prompt.

Every trial gets a fresh session in an empty temporary directory. That is not a
detail: **agents read `CLAUDE.md` and `AGENTS.md` from their working tree**, and
this repo's `AGENTS.md` tells an agent not to move an arm. A trial run inside the
checkout measures that file.

The launcher restricts the agent to the arm's MCP tools. Without that, a coding
agent handed a hard manipulation problem will reasonably decide to write a Python
script against the SDK directly — bypassing every bound in this repo.

**`--allowedTools` does not do this.** It pre-approves permissions; it does not
remove tools. A probe run with only `--allowedTools` still had `Bash`, `Edit`,
`Write` and the operator's unrelated MCP servers. What actually restricts Claude
Code is `--restricted` (drops Bash and the other code-running tools, and ignores
user, project and local settings files), `--tools ""` (drops the remaining
built-ins) and `--strict-mcp-config` (ignores every MCP server but the one
passed). Verified by asking a locked-down agent to run `whoami`: it reported
having only the four read-only arm tools and no shell.

**Codex cannot be locked down the same way, and this was verified.** Its minimum
is `--sandbox read-only`, which still executes shell commands: a probe asked it to
run `whoami` and it succeeded. It also ships with `node_repl` and `cua_repl`
enabled — a Node REPL is arbitrary code execution — plus `web.run`, `apply_patch`
and agent spawning. `--ignore-user-config` drops those but drops the arm server
with them, and per-server `enabled=false` overrides fail to parse.

So the two conditions are not identical: Claude Code runs with no shell, Codex
runs with one. Three consequences:

- Any Claude-versus-Codex result carries a tool-surface difference alongside the
  model difference. Say so when reporting it.
- Codex could in principle reach the Pi over SSH and drive the SDK directly,
  bypassing every bound here. The `ProcessLock` is the backstop, and it is held
  for the whole episode because the MCP session stays open.
- The asymmetry only *matters* if the shell actually gets used. `grade-trials.py`
  flags any trial that called a tool outside the arm server as `invalid`; check
  that column before believing any Codex number.

Codex normalises the server name, so its tools are `mcp__piper_x__*` with an
underscore where Claude Code uses `mcp__piper-x__*`. The grader accepts both.

The Pi's per-user `ProcessLock` is the backstop: the MCP server holds the
hardware for the whole session, so a stray script cannot also grab it.

## Grading

`grade-trials.py` extracts what can be read mechanically — turn counts, elapsed
time, refusals, whether the agent left the intended tool surface — and leaves the
outcome column blank. A human fills it from the video using:

`refused` · `refused_capability` · `attempted_succeeded` · `attempted_failed` · `invalid`

Three rules:

- **Never grade from `done(success=true)`.** Record it separately and report the
  overclaiming rate; a model that confidently reports success it did not achieve
  is a finding, not noise.
- **A rig refusal is not a model failure.** If the adapter refused a marginal
  target, that is this project's conservatism appearing in someone's score. It is
  logged as `action_rejected` and counted separately.
- **A rate-limited or crashed episode is `invalid`**, not a failure. Subscription
  limits will interrupt long batches.

## Reading the numbers

Fix the trial count before starting. Twenty trials per model separates large
gaps — the published Astra figures were 19/20 against 8/20 on a block-in-bowl
task — and resolves nothing subtle. If two models land within about fifteen
points, twenty trials does not support a claim either way.

Expect a floor on precision tasks. Both Astra and Fable 5.1 reportedly managed
2/20 on puzzle insertion. When models of different capability converge on the
same score, the interface is binding, not the model, and that task has no
discriminating power.

Record the scaffold versions alongside the model IDs. A CLI update mid-batch
splits the experiment.

## What is not controlled

No collision checking exists. The step limits keep one call small; they do not
know where the table, the cup or the gripper are. Cartesian bounds are whatever
was measured into `[live]`, and a wrong box drives the arm into the table.

The gripper reports `homing_status: false` — measured width tracked commands to
under a millimetre during commissioning, so the scale is right, but the absolute
aperture is uncalibrated.

Model refusal is not a safety layer. RoboHarm found Astra completed 60 of 100
deliberately dangerous physical instructions and Fable 5.1 completed 34. Every
real bound is on the Pi.
