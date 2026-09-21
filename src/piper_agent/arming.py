"""Time-boxed arming for physical motion.

A configuration flag is permanent, and a flag left on overnight is exactly the
hazard this project keeps designing against. Motion tools therefore exist only
inside a window an operator opened deliberately, at the rig, with a deadline.
The window lives in a temporary file so a reboot closes it.
"""

import json
import os
from pathlib import Path
import tempfile
import time

MAX_MINUTES = 60


def arm_path():
    # Overridable so tests and dry runs never touch the real rig's window.
    override = os.environ.get("PIPER_ARM_FILE")
    if override:
        return Path(override)
    return Path(tempfile.gettempdir()) / f"piper-x-agent-arm-{os.getuid()}.json"


def arm(minutes, note=""):
    if type(minutes) not in (int, float) or isinstance(minutes, bool) or not 0 < minutes <= MAX_MINUTES:
        raise ValueError(f"minutes must be greater than 0 and at most {MAX_MINUTES}")
    now = time.time()
    record = {"armed_at_unix_s": now, "expires_unix_s": now + float(minutes) * 60.0,
              "note": str(note)[:200]}
    path = arm_path()
    path.write_text(json.dumps(record))
    path.chmod(0o600)
    return record


def disarm():
    arm_path().unlink(missing_ok=True)


def status():
    """Return (armed, record or None, seconds_remaining)."""
    try:
        record = json.loads(arm_path().read_text())
        expires = float(record["expires_unix_s"])
    except (OSError, ValueError, KeyError, TypeError):
        return False, None, 0.0
    remaining = expires - time.time()
    if remaining <= 0:
        return False, record, 0.0
    return True, record, remaining


def require_armed():
    """Raise unless an operator has armed the rig; return seconds remaining."""
    armed, _, remaining = status()
    if not armed:
        raise RuntimeError(
            "Physical motion is not armed. An operator at the rig must run "
            "`piper-agent arm --minutes N` before motion tools accept commands."
        )
    return remaining
