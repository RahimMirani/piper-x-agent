import argparse
from contextlib import redirect_stdout
import importlib.util
import json
from pathlib import Path
import platform
import sys

from .config import Config


def main():
    parser = argparse.ArgumentParser(description="Piper X Codex tools; physical motion is unavailable in v0.1")
    parser.add_argument("command", choices=["doctor", "probe-cameras", "probe-arm", "snapshot", "snapshot-cameras", "serve", "camera-web", "motion-test", "lateral-test", "gripper-test"])
    parser.add_argument("--bind", default="0.0.0.0", help="camera-web bind address")
    parser.add_argument("--port", type=int, default=8090, help="camera-web TCP port")
    parser.add_argument("--joint", type=int, default=1, help="motion-test joint number (1-6)")
    parser.add_argument("--delta", type=float, default=0.02, help="motion-test/lateral-test offset in radians (max 0.2)")
    parser.add_argument("--timeout", type=float, default=8.0, help="motion-test convergence timeout")
    parser.add_argument("--confirm-motion-test", action="store_true", help="required before any physical motion")
    parser.add_argument("--config", type=Path, help="Explicit TOML configuration (required except doctor/probe-cameras)")
    args = parser.parse_args()
    try:
        if args.command == "doctor":
            print(json.dumps({"python": platform.python_version(), "system": platform.system(),
                              "architecture": platform.machine(),
                              "modules": {name: importlib.util.find_spec(name) is not None
                                          for name in ("mcp", "pyAgxArm", "pyorbbecsdk", "cv2", "numpy")},
                              "hardware_tested": False}, indent=2))
            return
        if args.command == "probe-cameras":
            from .cameras import enumerate_orbbec
            with redirect_stdout(sys.stderr):
                result = enumerate_orbbec()
            print(json.dumps(result, indent=2))
            return
        if not args.config:
            parser.error("--config is required; hardware mode is never selected implicitly")
        config = Config.load(args.config)
        from .runtime import Runtime
        if args.command in {"motion-test", "lateral-test", "gripper-test"}:
            if not args.confirm_motion_test:
                parser.error("physical tests require --confirm-motion-test")
            with redirect_stdout(sys.stderr), Runtime(config) as runtime:
                if args.command == "motion-test":
                    from .motion_test import run_single_joint_test
                    result = run_single_joint_test(config, args.joint, args.delta, args.timeout)
                elif args.command == "lateral-test":
                    from .motion_test import run_lateral_sweep
                    result = run_lateral_sweep(config, args.delta, args.timeout)
                else:
                    from .gripper_test import run_gripper_test
                    result = run_gripper_test(config)
                runtime.log(args.command.replace("-", "_"), result)
            print(json.dumps(result, indent=2, allow_nan=False))
        elif args.command == "camera-web":
            from .web import serve_camera_web
            with redirect_stdout(sys.stderr), Runtime(config) as runtime:
                serve_camera_web(runtime, args.bind, args.port)
        elif args.command == "serve":
            # Preserve fd 1 solely for MCP; vendor native/Python stdout goes to stderr.
            # Save a duplicate for the MCP transport before redirecting fd 1.
            import os
            protocol_stdout = os.fdopen(os.dup(sys.stdout.fileno()), "w", buffering=1, encoding="utf-8")
            os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
            sys.stdout = sys.stderr
            try:
                with Runtime(config) as runtime:
                    from .server import serve
                    serve(runtime, protocol_stdout)
            finally:
                protocol_stdout.close()
        else:
            with redirect_stdout(sys.stderr), Runtime(config) as runtime:
                if args.command == "probe-arm":
                    result = runtime.state()
                else:
                    result, _ = runtime.observe(include_arm=args.command != "snapshot-cameras")
                    result["run_directory_on_server"] = str(runtime.directory)
            print(json.dumps(result, indent=2))
    except (ImportError, OSError, ValueError, RuntimeError) as exc:
        print(f"piper-agent: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc


if __name__ == "__main__":
    main()
