import base64
import json


def create_server(runtime):
    import anyio
    from mcp.server.lowlevel import Server
    from mcp.types import ImageContent, TextContent, Tool, ToolAnnotations

    server = Server("piper-x-agent", version="0.1.0", instructions=(
        "Call robot_status first. v0.1 offers hardware telemetry and RGB observations, "
        "or explicitly labeled mock commands. It CANNOT move or stop the physical arm. "
        "Mock images are diagnostic patterns, not a scene. Never infer physical success from mock results. "
        "observe returns scene then wrist images and timestamps; views are not synchronized or calibrated. "
        "No autonomous enabling, homing, zeroing, firmware changes, or direct SDK writes. "
        "Keep logs/images on the Pi. A model response is not a real-time control loop."
    ))

    def tool(name, description, properties=None, readonly=True):
        properties = properties or {}
        return Tool(name=name, description=description,
                    inputSchema={"type": "object", "properties": properties,
                                 "required": list(properties), "additionalProperties": False},
                    annotations=ToolAnnotations(readOnlyHint=readonly, destructiveHint=False, openWorldHint=False))

    catalog = [
        tool("robot_status", "Get mode and capabilities without connecting to hardware."),
        tool("read_arm_state", "Read six joint angles in radians; requires advancing hardware feedback."),
        tool("observe", "Get fresh scene and wrist RGB images plus arm state. Saves local run files. No motion."),
    ]
    if runtime.config.mode == "mock":
        catalog.extend([
            tool("simulate_joint_move", "MOCK ONLY: set six joint angles, max 0.1 rad step per joint. No physics.",
                 {"joints_rad": {"type": "array", "items": {"type": "number"}, "minItems": 6, "maxItems": 6}}, False),
            tool("simulate_gripper", "MOCK ONLY: set jaw aperture in metres.",
                 {"aperture_m": {"type": "number", "minimum": 0, "maximum": 0.07}}, False),
            tool("simulate_stop", "Latch mock stop until server restart; NOT a physical stop tool.", readonly=False),
        ])

    @server.list_tools()
    async def list_tools():
        return catalog

    def execute(name, arguments):
        if name == "robot_status":
            result = runtime.status()
        elif name == "read_arm_state":
            result = runtime.state()
        elif name == "observe":
            metadata, frames = runtime.observe()
            content = [TextContent(type="text", text=json.dumps(metadata))]
            for frame in frames:
                content.append(TextContent(type="text", text=f"{frame.role} camera ({runtime.config.mode})"))
                content.append(ImageContent(type="image", mimeType=frame.mime,
                                            data=base64.b64encode(frame.data).decode("ascii")))
            return content
        elif name == "simulate_joint_move":
            result = runtime.simulate("move", arguments["joints_rad"])
        elif name == "simulate_gripper":
            result = runtime.simulate("grip", arguments["aperture_m"])
        elif name == "simulate_stop":
            result = runtime.simulate("stop")
        else:
            raise ValueError(f"Unknown tool: {name}")
        return [TextContent(type="text", text=json.dumps(result, allow_nan=False))]

    @server.call_tool()
    async def call_tool(name, arguments):
        if name not in {t.name for t in catalog}:
            raise ValueError(f"Tool unavailable in {runtime.config.mode}: {name}")
        try:
            return await anyio.to_thread.run_sync(execute, name, arguments)
        except Exception as exc:
            runtime.log("tool_error", {"tool": name, "error": str(exc)})
            raise

    return server


def serve(runtime, protocol_stdout):
    import anyio
    from mcp.server.stdio import stdio_server

    async def run():
        server = create_server(runtime)
        async with stdio_server(stdout=anyio.wrap_file(protocol_stdout)) as (read, write):
            await server.run(read, write, server.create_initialization_options())

    anyio.run(run)
