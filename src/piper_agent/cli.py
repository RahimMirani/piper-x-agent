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
    parser.add_argument("command", choices=["doctor", "probe-cameras", "probe-arm", "snapshot", "snapshot-cameras", "serve"])
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
        if args.command == "serve":
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
