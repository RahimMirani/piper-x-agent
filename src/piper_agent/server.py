import base64
import json


_READONLY_INSTRUCTIONS = (
    "Call robot_status first. This mode offers hardware telemetry and RGB observations, "
    "or explicitly labeled mock commands. It CANNOT move or stop the physical arm. "
    "Mock images are diagnostic patterns, not a scene. Never infer physical success from mock results. "
    "observe returns scene then wrist images and timestamps; views are not synchronized or calibrated. "
    "No autonomous enabling, homing, zeroing, firmware changes, or direct SDK writes. "
    "Keep logs/images on the Pi. A model response is not a real-time control loop."
)

_LIVE_INSTRUCTIONS = (
    "Call robot_status first, then observe before and after every movement. This mode moves a "
    "REAL arm. Motion tools accept commands only while an operator has armed the rig, and the "
    "window expires. All targets are ABSOLUTE, never relative, and each call may move the arm "
    "only a short bounded step, so reach a distant pose with several calls. "
    "A refused command means the target was out of range or too far in one step; nothing moved, "
    "so correct the target and retry rather than repeating it. "
    "There is NO collision checking: the bounds keep one call small, they do not know where the "
    "table, the cup or your own gripper are. Look at the images before every move. "
    "stop is a damped software stop, not a hardware emergency stop. "
    "Call done when the task is finished or you cannot proceed; a human reviews the video and "
    "decides the outcome, so report honestly rather than optimistically."
)


def create_server(runtime):
    import anyio
    from mcp.server.lowlevel import Server
    from mcp.types import ImageContent, TextContent, Tool, ToolAnnotations

    live = runtime.live
    server = Server("piper-x-agent", version="0.1.0",
                    instructions=_LIVE_INSTRUCTIONS if live else _READONLY_INSTRUCTIONS)

    def tool(name, description, properties=None, readonly=True, required=None):
        properties = properties or {}
        return Tool(name=name, description=description,
                    inputSchema={"type": "object", "properties": properties,
                                 "required": list(properties) if required is None else required,
                                 "additionalProperties": False},
                    annotations=ToolAnnotations(readOnlyHint=readonly, destructiveHint=False, openWorldHint=False))

    catalog = [
        tool("robot_status", "Get mode and capabilities without connecting to hardware."),
        tool("read_arm_state", "Read six joint angles in radians; requires advancing hardware feedback."),
        tool("observe", "Get fresh scene and wrist RGB images plus arm state. Saves local run files. No motion."),
        tool("observe_cameras", "Get both fresh RGB images without connecting to the arm. Use for camera setup."),
    ]
    if live:
        # Motion tools are listed only when an operator has already armed the
        # rig, so an unarmed session cannot see a way to move the arm. Every
        # call re-checks the window, because it can expire mid-session.
        from . import arming
        if arming.status()[0]:
            catalog.extend([
                tool("move_joints",
                     "Move to an ABSOLUTE six-joint pose in radians. Bounded step per call; "
                     "out-of-range targets are refused, not clamped. Returns the MEASURED pose.",
                     {"joints_rad": {"type": "array", "items": {"type": "number"},
                                     "minItems": 6, "maxItems": 6}}, False),
                tool("move_to_pose",
                     "Move the flange to an ABSOLUTE Cartesian pose [x, y, z, roll, pitch, yaw] "
                     "in the arm base frame (metres, radians). Bounded step per call; poses "
                     "outside the configured workspace are refused. Returns the MEASURED pose.",
                     {"pose": {"type": "array", "items": {"type": "number"},
                               "minItems": 6, "maxItems": 6}}, False),
                tool("set_gripper",
                     "Set the jaw opening in metres at a low force. Returns the MEASURED width.",
                     {"width_m": {"type": "number", "minimum": 0, "maximum": 0.07}}, False),
                tool("stop",
                     "Request a damped software stop. NOT a hardware emergency stop.", readonly=False),
                tool("done",
                     "Declare the episode finished. Record whether you believe you succeeded and "
                     "why. A human grader reviews the video; this does not decide the outcome.",
                     {"success": {"type": "boolean"},
                      "note": {"type": "string"}}, False, required=["success"]),
            ])
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
        elif name in {"observe", "observe_cameras"}:
            metadata, frames = runtime.observe(include_arm=name == "observe")
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
        elif name == "move_joints":
            result = runtime.act("move_joints", arguments["joints_rad"])
        elif name == "move_to_pose":
            result = runtime.act("move_to_pose", arguments["pose"])
        elif name == "set_gripper":
            result = runtime.act("set_gripper", {"width_m": arguments["width_m"]})
        elif name == "stop":
            result = runtime.act("stop")
        elif name == "done":
            result = runtime.declare_done(arguments["success"], arguments.get("note", ""))
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
