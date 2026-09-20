# Hardware evidence and outstanding checks

Recorded from the user's existing `trc/repos/piper-x-arm` checkout on 2026-09-13.
These are historical notes, not measurements from the current disconnected rig.
See [Pi camera commissioning](pi-camera-check.md) for the subsequently verified
camera installation and RGB checks. The remaining historical entries below
still describe the earlier checkout.

No private network addresses, credentials, old recordings, or vendor binaries
were copied into this repository.

| Item | Historical evidence | Current status |
| --- | --- | --- |
| Host | User plans Raspberry Pi 5 | OS, RAM, power and SSH not confirmed |
| Scene camera | Orbbec DaBai DC1; serial `CC1N16200TF` | Confirm by enumeration and lens-cover test |
| Wrist camera | Orbbec DaBai DC1; serial `CC1N16200F9` | Confirm by enumeration and lens-cover test |
| Camera SDK | Legacy wrapper v1, OrbbecSDK 1.10.35 | Pi ARM64/Python build still to be selected |
| Arm SDK | pyAgxArm, `ArmModel.PIPER_X` | Revision still to be selected and tested |
| Firmware profile | `v188` for the earlier S-V1.8-x arm | Actual firmware still to be checked |
| Adapter | gs_usb/candleLight, 1 Mbps | Confirm USB device, kernel driver and CAN interface |

Sources inside the old checkout:

- `scripts/camera_check.py`: DC1 identity, legacy SDK requirement, approximately
  0.3 m minimum depth range reported for that setup.
- `scripts/camera_live_browser.py`: two DC1 streams and MJPG color decoding.
- `NOTION_PiPER_X.md`, section 8: scene/wrist serial mapping.
- `REAL_ARM.md` and `piper_x_twin/backends.py`: SDK/firmware settings and previous
  control behavior.

The old notes include both SocketCAN setup and a statement about direct gs_usb
access. This project explicitly requests `interface="socketcan"`; verify the
actual adapter/SDK path rather than treating those notes as interchangeable.

The old CPV streaming code had hardware-specific wrist sign corrections. Those
corrections are not copied here, and this project does not implement CPV or MIT
control. The old startup also enabled and homed the arm automatically. This
project only connects and reads advancing joint feedback.

Depth measurements close to the gripper may be invalid even if color is useful.
This release uses RGB only. Camera intrinsics, scene-to-base and wrist-to-tool
transforms, depth alignment, joint zeroing and gripper geometry remain unverified.
