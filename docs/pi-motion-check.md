# Pi arm commissioning — 2026-09-20

First supervised physical motion of the PiPER-X on the user's Pi, with the
operator present at the rig and the workspace clear. Performed after the camera
commissioning recorded in [pi-camera-check.md](pi-camera-check.md).

No homing, re-zeroing, calibration, firmware change or fault clearing was
performed. The gripper was not calibrated.

## Verified environment

- `can0` up on the gs_usb/candleLight adapter (`1d50:606f`), bitrate 1 Mbps,
  sample point 0.75, state ERROR-ACTIVE.
- Arm answers as `ArmMsgFeedbackStatusV188`, consistent with the `v188` firmware
  profile in `config/local.toml`. This confirms the profile is accepted, not
  that it is the only correct one.
- Before any command: `arm_status NORMAL`, `ctrl_mode STANDBY`, `err_code 0`, no
  joint angle-limit or joint communication errors, all six joints disabled.
- Gripper feedback present at ~200–210 Hz, driver enabled, no FOC fault bits.
- All 16 project tests passed on the Pi, including the MCP stdio test.
- A full `snapshot` returned both RGB frames plus joint state in 1.45 s, scene
  `CC1N16200TF` and wrist `CC1N16200F9`, pair skew 23 ms.

## Defects found and fixed before trusting any result

1. **Stale feedback was accepted as a result.** `_joint_feedback` returned on the
   first readable message, because `previous` was `None` on the first pass. The
   SDK hands back a cached frame, so `_wait_target` could converge on a reading
   taken before the command was sent. The run recorded at `06:36:28Z` reported a
   pass whose outward pose was byte-identical to its start pose; the arm had not
   moved. Both helpers now require the SDK timestamp to advance during the call,
   matching what `ReadOnlyArm.state()` already did.
2. **`move_j` clamps silently.** `Validator.clamp_joints` clamps every target into
   the model's joint limits and only prints a warning. With motors off this arm
   rests at joint 2 ≈ −0.038 rad and joint 3 ≈ +0.040 rad, just outside the
   PiPER-X ranges `[0, π]` and `[−2.967, 0]`, so a single-joint test on joint 1
   was also, silently, commanding joints 2 and 3 to zero. Refusing outright would
   block every test at this resting pose. The baseline is now clamped knowingly,
   the shift is reported as `sdk_clamp_shift_rad`, and an excursion beyond
   0.05 rad still refuses to move.
3. **The gripper test verified nothing.** It commanded three widths and returned
   success without reading one feedback frame. It now requires advancing gripper
   feedback, checks the FOC fault bits and driver-enable state, records measured
   width and force at each step, and fails below 10 mm of measured travel.

`robot.reset()` was also removed from the lateral sweep. The single-joint test
never needed it, and it is an extra motion-controller state change before enable.

## Physical checks performed

All at `set_speed_percent(10)` in joint-space mode, each returning to its
starting pose. Motion amplitude was raised from the earlier 0.02 rad cap to
0.2 rad at the operator's direction, so the movement is visible to a supervisor.

| Check | Commanded | Measured | Return error |
| --- | --- | --- | --- |
| Single joint, J1 | 0.02 rad | 0.019862 rad | 0.0012 rad |
| Single joint, J1 | 0.2 rad | 0.198810 rad | 0.0017 rad |
| Lateral sweep, J1 | ±0.2 rad (0.4 total) | 0.398005 rad | 0.0008 rad |
| Gripper, 0.02 → 0.04 → 0.02 m | 0.02 m travel | 0.0196 m | n/a |

Gripper steps tracked their commands to within 0.7 mm (0.0197 / 0.0393 / 0.0201 m
measured) at 1.0 N. The first physical `move_j` straightened joints 2 and 3 by
0.038 and 0.040 rad to their mechanical zero, as the clamp above predicts; they
stayed at zero for the rest of the session while motors were enabled.

After the session: `arm_status NORMAL`, `err_code 0`, no error bits, all six
joints enabled and holding, gripper at 0.0201 m. The tests deliberately leave the
arm enabled, because disabling a raised arm lets it drop.

## Saved evidence on the Pi

- `runs/20260920T070627Z-670e2541/` — J1 at 0.02 rad.
- `runs/20260920T070655Z-eb81a67f/` — J1 at 0.2 rad.
- `runs/20260920T070722Z-79259b5a/` — lateral sweep at 0.2 rad.
- `runs/20260920T070750Z-a0d899a1/` — gripper test.
- `runs/20260920T070350Z-08c12b0b/` — full two-camera observation with joint state.

Run names use UTC.

## Still outstanding

- The gripper reports `homing_status: false`, so the controller does not claim a
  zero reference. Measured width tracked commands to under 1 mm here, which shows
  the scale is right, but absolute aperture is not calibrated and no grasp force
  has been characterised. Nothing was gripped.
- Joint signs and zeros are not established; only joint 1 was moved under command.
  Joints 2–6 have never been commanded away from their current pose.
- No collision checking exists. These are bounded single-joint steps in clear
  space, not planned trajectories. A joint step limit is not collision avoidance
  and `electronic_emergency_stop()` is not a hardware E-stop.
- Tool geometry, mounting, table/workspace bounds and camera-to-base calibration
  are all unverified, so no Cartesian motion should be attempted yet.
- `motion_status` only separates reach-success from reach-failure; it is not a
  busy flag, and it reads `REACH_TARGET_POS_SUCCESSFULLY` even in standby.
  Convergence here rests on measured position error and measured displacement.
- Physical motion remains outside the MCP server. The model cannot move this arm
  through any tool in this release; these tests are CLI-only and require
  `--confirm-motion-test`.
