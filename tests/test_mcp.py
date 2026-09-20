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
