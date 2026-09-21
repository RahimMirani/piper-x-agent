"""Actual stdio round trip. Runs on Pi/CI with MCP installed; otherwise skipped."""
import asyncio
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest


@unittest.skipUnless(importlib.util.find_spec("mcp"), "Install project on Pi/CI for MCP transport tests")
class MCPTests(unittest.TestCase):
    def test_stdio_images_commands_and_hardware_tool_catalog(self):
        asyncio.run(asyncio.wait_for(self.scenario(), 60))

    async def scenario(self):
        from mcp import ClientSession, StdioServerParameters
        from mcp.client.stdio import stdio_client
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as tmp:
            for mode in ("mock", "hardware_readonly"):
                config = Path(tmp) / f"{mode}.toml"
                config.write_text(f'mode="{mode}"\nrun_directory="runs"\n[cameras]\nscene_serial="a"\nwrist_serial="b"\n')
                parameters = StdioServerParameters(command=sys.executable,
                    args=["-m", "piper_agent.cli", "serve", "--config", str(config)],
                    env={**os.environ, "PYTHONPATH": str(root / "src")})
                async with stdio_client(parameters) as (read, write):
                    async with ClientSession(read, write) as session:
                        await asyncio.wait_for(session.initialize(), 15)
                        names = {t.name for t in (await session.list_tools()).tools}
                        status = await session.call_tool("robot_status", {})
                        self.assertFalse(status.isError)
                        self.assertEqual(json.loads(status.content[0].text)["mode"], mode)
                        if mode == "hardware_readonly":
                            self.assertEqual(names, {"robot_status", "read_arm_state", "observe", "observe_cameras"})
                            denied = await session.call_tool("simulate_joint_move", {"joints_rad": [0] * 6})
                            self.assertTrue(denied.isError)
                            continue
                        observation = await session.call_tool("observe", {})
                        self.assertFalse(observation.isError)
                        self.assertEqual(len([c for c in observation.content if c.type == "image"]), 2)
                        cameras = await session.call_tool("observe_cameras", {})
                        self.assertFalse(cameras.isError)
                        self.assertIsNone(json.loads(cameras.content[0].text)["state"])
                        self.assertEqual(len([c for c in cameras.content if c.type == "image"]), 2)
                        rejected = await session.call_tool("simulate_joint_move", {"joints_rad": [2] * 6})
                        self.assertTrue(rejected.isError)
                        malformed = await session.call_tool("simulate_joint_move", {"joints_rad": [0] * 5})
                        self.assertTrue(malformed.isError)
                        moved = await session.call_tool("simulate_joint_move", {"joints_rad": [0.05] * 6})
                        self.assertFalse(moved.isError)
                        self.assertFalse((await session.call_tool("simulate_stop", {})).isError)
                        self.assertTrue((await session.call_tool("simulate_gripper", {"aperture_m": 0.02})).isError)
            await self.live_catalog(root, tmp)

    async def live_catalog(self, root, tmp):
        """Motion tools exist only inside an armed window. Touches no hardware."""
        from mcp import ClientSession, StdioServerParameters
        from mcp.client.stdio import stdio_client
        config = Path(tmp) / "live.toml"
        config.write_text('mode="hardware_live"\nrun_directory="runs"\n'
                          '[cameras]\nscene_serial="a"\nwrist_serial="b"\n')
        window = Path(tmp) / "arm.json"
        readonly = {"robot_status", "read_arm_state", "observe", "observe_cameras"}
        motion = {"move_joints", "move_to_pose", "set_gripper", "stop", "done"}

        for armed in (False, True):
            if armed:
                import time as _time
                window.write_text(json.dumps({"armed_at_unix_s": _time.time(),
                                              "expires_unix_s": _time.time() + 300}))
            parameters = StdioServerParameters(command=sys.executable,
                args=["-m", "piper_agent.cli", "serve", "--config", str(config)],
                env={**os.environ, "PYTHONPATH": str(root / "src"),
                     "PIPER_ARM_FILE": str(window)})
            async with stdio_client(parameters) as (read, write):
                async with ClientSession(read, write) as session:
                    await asyncio.wait_for(session.initialize(), 15)
                    names = {t.name for t in (await session.list_tools()).tools}
                    status = json.loads((await session.call_tool("robot_status", {})).content[0].text)
                    self.assertTrue(status["physical_motion_supported"])
                    self.assertFalse(status["collision_checking"])
                    self.assertEqual(status["motion_armed"], armed)
                    if not armed:
                        # Unarmed: no motion tool is even listed, and calling
                        # one anyway is refused rather than reaching the arm.
                        self.assertEqual(names, readonly)
                        self.assertTrue((await session.call_tool(
                            "move_joints", {"joints_rad": [0] * 6})).isError)
                    else:
                        self.assertEqual(names, readonly | motion)
                        # done records a claim and touches no hardware.
                        done = await session.call_tool("done", {"success": False, "note": "n/a"})
                        self.assertFalse(done.isError)
                        self.assertFalse(json.loads(done.content[0].text)["claimed_success"])
