# MCP Agent (Model Context Protocol)

RobotCAD provides an **MCP server** that lets an external LLM agent
(for example, an agent in VS Code — Roo, Cline, Continue — or Claude Desktop)
control the RobotCAD document: create robots, links, joints and collisions,
position and rotate objects, set materials, compute mass and inertia,
measure distances and radii, select objects and take 3D view snapshots.

## Quick start

1. Open FreeCAD with the RobotCAD workbench loaded.
2. On the toolbar, press **MCP Agent** (a robot-with-connector icon).
3. In the dialog press **Start server** — an HTTP MCP server starts on
   `http://127.0.0.1:8006/mcp`.
4. Copy the config (**Copy HTTP config** or **Copy stdio config**) and add it
   to your agent's settings.

### VS Code (`.mcp.json`)

> **Important:** Roo / Cline expect `mcpServers` as an **object** keyed by the
> server name, and the HTTP transport type must be `streamable-http`
> (not `http`). The value `"type": "http"` is rejected with the error
> `Invalid MCP settings format: mcpServers.robotcad: Invalid input`.

```json
{
  "mcpServers": {
    "robotcad": {
      "type": "streamable-http",
      "url": "http://127.0.0.1:8006/mcp",
      "disabled": false
    }
  }
}
```

### stdio (for agents that do not support the HTTP transport)

```json
{
  "mcpServers": {
    "robotcad": {
      "type": "stdio",
      "command": "<python-exe>",
      "args": ["<path-to-workbench>/freecad/cross/mcp/stdio_server.py"],
      "env": {
        "MCP_FREECAD_URL": "http://127.0.0.1:8006/mcp"
      }
    }
  }
}
```

> The path to the `stdio_server.py` script (inside the workbench at
> `freecad/cross/mcp/stdio_server.py`) can be copied from the
> **MCP Agent** dialog (the **Copy stdio config** button).

> **Important:** FreeCAD must be running and the HTTP server enabled
> (the **Start server** button) while the agent uses the tools.

## Transports

| Transport | Description |
|---|---|
| **Streamable HTTP** | The server runs inside the FreeCAD process in a background thread. External agents connect via `http://127.0.0.1:8006/mcp`. |
| **stdio** | A separate bridge process `freecad/cross/mcp/stdio_server.py` that connects to the FreeCAD HTTP server and re-exposes the same tools over stdio. |

## Tools

All tools operate on the **active document** and accept objects by name
(`Name` or `Label`).

### Creation

| Tool | Description |
|---|---|
| `create_document(name)` | Create a new FreeCAD document and make it active (useful when there is no active document yet). |
| `create_robot(name)` | Create an empty `Cross::Robot`. |
| `create_link(robot, name)` | Create a `Cross::Link` and add it to the robot. |
| `create_links_filled(robot, object_names)` | Create links filled with Real/Visual from existing objects. |
| `create_joint(robot, name, parent_link, child_link, type, lower, upper, effort, velocity)` | Create a joint between two links and set type/limits. The joint's local Z is oriented later with `rotate_object` (there is no `axis` parameter). |
| `create_joints_filled(robot, link_names_in_order, connect_type)` | Create joints in a chain (`chain`) or "spider" (`spider` — all links to the first one). |

### Collisions

| Tool | Description |
|---|---|
| `create_collisions(link_or_robot, type)` | Create collisions for **several** links/robots in ONE call. `link_or_robot` is a **list** of names/Labels; `type` is one type for all, or a **list parallel** to `link_or_robot`. **Call it without `type` by default** — an exact geometry copy (`copy`, the default). Primitives (`box`, `sphere`, `cylinder_x/y/z`, from the bounding box) **only if the user explicitly asks**. A **robot** covers the robot **as a whole**; its links passed in the same call are removed first (no duplicates). Always use this batch tool, never once per link. |

### General workflow

Before touching the document, write the plan of the General algorithm as a
**TODO list** and work through it in order, marking each item done as you
complete it. Following the algorithm means following that TODO list — do not
skip, reorder or batch its items (see the `general_algorithm` topic of the
`instructions_to_work_with_tools` tool, the source of truth).

### Positioning and rotation

**The primary positioning method is `set_placement_between`.** After the joints
are created, snap the contact zones of two neighbouring links: one reference (a
face, edge, vertex or circle) on the parent link and one on the child link, then
call `set_placement_between(target, ref1, ref2)` (only the two references go
into the selection). Take a control snapshot (`get_snapshot`); if a link is
oriented incorrectly, correct it with `rotate_object` and take another snapshot.
Repeat for the next links.

> **If the call succeeds but the target did NOT move**, both references most
> likely resolved to the same point. Verify with `get_objects_info([target])`
> and pick different ones (e.g. a face instead of a vertex).

**The two references are joined BY THEIR CENTERS:** the center of `ref2` is
brought onto the center of `ref1` (face/circle = geometric center, edge =
midpoint, vertex = itself). Pick references so the points that must touch are
exactly those centers (e.g. a hole center on the parent, the mating boss center
on the child).

> **A camera (or similar forward-looking) sensor link is NEVER joined with
> `set_placement_between`.** Use **only**
> `set_placement_between_sensor_link_and_parent`: it joins the two links by the
> centers of `ref_parent`/`ref_sensor` **and** turns the parent joint so the
> sensor's front follows its local X (red arrow) = the look direction.
> `set_placement_between` leaves such a sensor mis-oriented. EXCEPTION: a link
> hosting only a **lidar mounted HORIZONTALLY** (scanning all around) **is**
> positioned with `set_placement_between`, then its **parent joint** is rotated
> about **Z** until the sector's front (the joint's positive X) points where it
> must.

**Joint axis orientation.** A `revolute`/`continuous` joint **rotates around its
local Z** (blue arrow); a `prismatic` joint **moves along its local Z**.
Rotating a joint also rotates its **child chain and end link**, so orient a
wheel/arm by rotating the JOINT, never the link. The child link's functional
axis (e.g. a wheel's axle) must lie along the joint's local Z, otherwise the
joint turns the link about the wrong axis and the wheel will not roll. There is
no `axis` parameter on `create_joint` — use `rotate_object(joint, axis, angle)`.

> **Sides are defined as seen from BEHIND the object.** The LEFT/RIGHT side of
> every part (wheels, arms, sensors, ...) is the side you see when you stand
> **behind** the object and look along its **forward** direction: what lies on
> your left is LEFT, on your right is RIGHT. Take the forward direction from the
> robot's direction-defining parts (forward-looking sensors, end effector); with
> no such feature use the base convention (local +X forward, so +Y is left).
> Never derive a side from an arbitrary axis, from the current snapshot's
> viewpoint or from the order the links were created.

> **Chassis: all wheel Z axes must point the same way (to the LEFT).** All
> wheels' joint local Z axes (the axles) MUST be parallel and point the SAME
> way — to the LEFT **as seen from behind the chassis**. Never mirror the joints
> into opposite directions; flip a wrong one by rotating its JOINT.

**Mirroring a link to the other side of the parent** — the one case where
rotating the link is correct (the joint and link are oriented correctly but the
body penetrates the parent through its full height). Rotate the **link** 180°
about **X**:

```
rotate_object(link_name, 'x', 180)
```

The link is aligned along its local Z, so an X rotation mirrors it about Z: it
flips to the other side while its Z alignment and the joint orientation stay
intact. Use **X, not Z** (a Z rotation only spins it around its own functional
axis).

> **MANDATORY OVERLAP CHECK — HARD GATE, NEVER SKIP, ONE CHILD AT A TIME, PRINT
> THE NUMBERS.** Run it for EACH child ONLY AFTER its joint is oriented (for a
> multi-wheel parent: after ALL wheels' joint Z axes are parallel, pointing
> left) and after EVERY later `rotate_object` on that joint — before collisions
> and materials. Orientation changes the child's global ranges, so an earlier
> check is meaningless. Never batch it and never assume symmetry: symmetric
> children on opposite corners commonly end up on OPPOSITE sides — some outside
> (correct), some inside (overlapping, wrong). On a 4-wheeled chassis the two
> wheels at one end sit outside, the two at the other end sit inside the body
> and MUST be mirrored. Per child:
> 1. `get_objects_info([child_link, parent_link])` — read the GLOBAL `Geometry`
>    (`center_of_mass`, face `center_of_mass`, `vertices[*].point`) and compute
>    min/max on EACH of X, Y and Z for both.
> 2. **Check ALL THREE AXES, not just one.** On every axis the child's range
>    must extend OUTSIDE the parent's; if it is CONTAINED on ANY axis, the
>    bodies overlap → OVERLAP FAILURE. One convenient axis is NOT enough: a
>    wheel snapped to a corner can pass along X while its disc is buried inside
>    the parent along Y. Do not judge by eye.
> 3. Mirror on failure: `rotate_object(child_link, 'x', 180)`, then re-check and
>    confirm the child's range now lies OUTSIDE on the failing axis.
> 4. PRINT the evidence for EVERY child before continuing:
>    `overlap check <child>: X[..] Y[..] Z[..] vs parent X[..] Y[..] Z[..] ->
>    OUTSIDE` (or `OVERLAP -> mirrored`).
>
> An overlapping child is a positioning FAILURE. Do NOT proceed to the next
> child, to collisions or to materials until EVERY child has passed AND its
> numbers have been printed.

**Do not create LCS objects** — plain subelement references are enough;
`create_lcs` only if the user explicitly asks for it.

> `set_object_placement` is disabled in the tool registry (implementation kept
> in [`tools_registry.py`](../freecad/cross/mcp/tools_registry.py), commented
> out in `TOOLS`). `set_placement_between` replaces it.

**A robot link cannot be a reference** — only a face, edge, vertex, circle or
LCS on the Real element of a link. Format:
`<real_link>.<inner_link_name>.<feature>.<subelement>`, e.g.
`real_l_chassis001_.chassis001.Box.Face3` (`<real_link>` = the Real element link
`real_l_...`; `<inner_link_name>` = the **Name of the `App::Link` inside the Real
element**, e.g. `chassis001`, **NOT** the source body name; `Box` = feature;
`Face3` = subelement). The path **must** start with the Real element link; a
path from the robot link (`l_...`) is rejected. It is tolerant: the inner link
may be `Name` or `Label`, intermediate levels may be omitted, `real_l_...` is
accepted though not required. If it cannot be resolved, the error lists the
tried variants.

> **Common mistake:** using the source body name (`chassis`, `wheel`) instead of
> the inner link Name (`chassis001`, `wheel001`). Inspect the Real element with
> `get_objects_info(['real_l_...'])` to read the inner link `Name` first.

| Tool | Description |
|---|---|
| `set_placement_between(target, ref1, ref2, move)` | **Primary positioning method** (analog of `Set Placement - fast`). Joins the two references by their CENTERS; only `ref1`/`ref2` go into the selection (not `target`); one on the parent, one on the child. `move` defaults to `leaf` (final chain element only). NEVER on a camera-like sensor link — EXCEPTION: a link hosting only a **horizontal lidar** IS positioned with this tool, then its parent joint is rotated about **Z**. |
| `measure_geometry(ref1, ref2, mode)` | **Exact measurement** in millimetres. With `ref2`: the **distance** between two faces/edges/vertices — `distance` (real minimum gap between the surfaces/curves/points), `center_distance`, per-axis `delta` and the `closest_points`. Without `ref2`: the **radius** of a circular edge (rim, hole) or of a cylindrical/spherical/toroidal face — `radius`, `diameter`, `center`, `axis`. `ref1`/`ref2` MUST be a **full subelement path** in the same format as `set_placement_between` (`<real_link>.<inner_link_name>.<feature>.<subelement>`, e.g. `real_l_wheel001_.wheel001.Cylinder.Face2`) or an LCS. A bare object name (robot link `l_...`, Real element `real_l_...`, whole object) is **rejected** with a hint naming the exact prefix — such objects have no geometry of their own. Use it for every value that must be exact (gaps, wheel separation, radii, offsets) instead of estimating from bounding boxes. |
| `set_placement_between_sensor_link_and_parent(sensor, ref_parent, ref_sensor)` | Position a **link**-attached **forward-looking** sensor (camera, vertical lidar). **The only tool that may connect such a sensor link to its parent link**: joins them by the CENTERS of the references, then turns the parent joint so the sensor's front follows its local X (red arrow) = the look direction. The roll about X must still be corrected. `sensor` may be the sensor object **or the sensor link that hosts it** (e.g. `l_camera001`) — both are accepted. |
| `set_placement_vision_mode(robot)` | Show only Real elements, hide Visual and Collision. Call before positioning. |
| `rotate_object(object_name, axis, angle_deg)` | Rotate the joint Origin / link MountedPlacement / LCS — corrects orientation after a control snapshot. |
| `create_lcs(link, subelement)` | Create an LCS on a face/edge/circle/vertex of the link Real element. **Only if the user explicitly asks** — plain subelement references are enough. |

### Material and inertia

| Tool | Description |
|---|---|
| `set_material(object_name, material_card_path, density, card_name)` | Set a material on a link/robot (`.FCMat` or density). |
| `calculate_mass_and_inertia(robot_or_link_names)` | Compute mass, inertia and center of mass. |
| `set_joint_values(robot, values)` | Set joint values (degrees/mm). |

### Controllers and broadcasters (ros2_control)

A robot hosts `Cross::Controller` and `Cross::Broadcaster` objects built from
the `ros2_controllers` submodule. Add them when the user asks for them and, in
the general flow, give the robot the controllers its **purpose** and its
**actuated joints** require.

**How to choose (by the robot's purpose):**

- Any robot with actuated joints MUST get a `joint_state_broadcaster`
  (`kind='broadcaster'`).
- A chassis/mobile base gets a drive controller matching its wheels:
  `diff_drive_controller` (differential), `ackermann_steering_controller`
  (car-like) or an omni/swerve controller — set its `*_wheel_names` /
  `steering_*_names` and frame parameters.
- **Left/right wheels**: a side is defined **as seen from BEHIND the robot** —
  stand behind it and look along its FORWARD direction; what lies on your left
  is LEFT, on your right is RIGHT. Take the FORWARD direction from meaningful
  features (forward-looking sensors, the end effector); with no such feature,
  fall back to the chassis convention (local +X forward, so +Y is left). Never
  derive a side from an arbitrary axis or from the snapshot viewpoint.
- **Wheel separation**: measure it as the distance between the **nearest facing
  inner faces** of the right and left wheel — not between their centers and not
  between their outer faces. Obtain it with
  `measure_geometry(<right wheel inner face>, <left wheel inner face>)` and use
  the returned `distance`.
- **Wheel radius**: obtain it with `measure_geometry(<wheel rim edge or
  cylindrical face>)` (no `ref2`) and use the returned `radius`/`diameter`.
- **Parameter units**: distance parameters are in **metres** unless the
  parameter's description says otherwise. Convert the model measurement to the
  parameter's unit before setting it; check the `description` from
  `list_controller_parameters(...)`.
- **Measure, do not guess**: whenever a controller parameter needs a geometric
  value (wheel separation, wheel radius, track/offset, a mounting distance),
  obtain it with `measure_geometry(...)` — `distance` for a gap between two
  faces/edges/vertices, `radius` for a circular edge or cylindrical face — and
  convert the millimetre result to the parameter's unit. Never derive such a
  value from a bounding box, from `center_of_mass` differences or by eye.
- A manipulator arm gets a `joint_trajectory_controller` over its joints (set
  `joints` and `base_frame_id`); a gripper gets `gripper_action_controller` (or
  a position/trajectory controller) over the gripper joints; any other
  functional group (pan-tilt, lift, ...) gets a suitable trajectory / velocity /
  position controller.
- Check `list_controller_types()` before choosing.

| Tool | Description |
|---|---|
| `list_controller_types()` | Available controllers and broadcasters (`name`, `description`, `plugin_class_name`, `parameter_count`). |
| `create_controller(robot, type, name, kind)` | Add a controller (or broadcaster). `type` is the ROS controller name (e.g. `diff_drive_controller`) or the plugin class full name; `kind` is `'controller'` (default) or `'broadcaster'`; `name` overrides the Label. |
| `get_robot_controllers(robot)` | List the controllers and broadcasters present on a robot. |
| `list_controller_parameters(controller)` | List a controller's parameters: `full_name`, `type_fc`, `type`, `default_value`, `description`, `current_value`. `current_value` is always JSON: a link/joint parameter reports the object **name** (pass it back to `set_controller_parameter`), a vector reports `[x, y, z]`, a placement reports `{x, y, z, q0, q1, q2, q3}` and a quantity reports its plain number. |
| `set_controller_parameter(controller, parameter, value)` | Set one parameter by full name. Nested names are joined with `'___'`. Joint/link parameters (e.g. `joints`, `base_frame_id`) accept an object name or a list of names. |
| `set_controller_parameters(controller, values)` | Set several parameters at once (`full_name` -> value). |

Typical workflow: create the robot with its links/joints first, identify the
functional joint groups, add `joint_state_broadcaster`, then per group
`list_controller_types()`, `create_controller(robot, '<type>')`,
`list_controller_parameters('<controller>')` and
`set_controller_parameter('<controller>', '<param>', <value>)`.

### Sensors

A robot hosts `Cross::Sensor` objects (Gazebo sensors from `resources/sensors`),
attached to a `Cross::Link`, `Cross::Joint` or `Cross::Robot`. A sensor has **no
position of its own**: it is oriented by the parent joint of the element it is
attached to (X = look direction, Z = up, Y = left). Adding sensors is a
**mandatory** part of the general algorithm — never skip it. Choose the set
FIRST; the three cases are mutually exclusive and checked in this order:

1. **The user asked something specific** (type, count, placement) → add exactly
   that, INSTEAD of the default set.
2. **The source objects already contain specific sensors** (a `Cross::Sensor`, a
   sensor link, or a plain camera/lidar part) → add THOSE INSTEAD of the default
   set.
3. **The user said nothing about sensors** and you follow the general algorithm
   → add the DEFAULT SENSOR SET below.

Attach each sensor where it **belongs by meaning** — a camera on the head /
end-tool link, a lidar on the chassis, a wheel encoder on the wheel's joint, an
IMU on the base; never the root link by default.

**Default sensor set (case 3):**

- **RGB-D camera** (`rgbd_camera`) on the chassis, looking FORWARD.
- **360-degree lidar** (`gpu_lidar`, circular scan) on the chassis.
- **RGB-D camera** (`rgbd_camera`) on the manipulator, aimed at the END TOOL
  (skip if there is no manipulator).

No chassis → put the chassis sensors on the base link. Adapt the set to what the
robot actually has.

**Check the sensor frame orientation (X = look, Y = left, Z = up).** After
positioning, ALWAYS verify the frame of the sensor's **parent joint** — MANDATORY
and the step most often skipped, leaving the image LYING ON ITS SIDE. Check **ALL
THREE axes — X, Y AND Z — one by one**; do NOT stop after X and Z:

- **X (red) = the look direction**;
- **Z (blue) = the top of the frame** — **perpendicularly UP relative to the
  parent part**, NOT relative to the world and NOT along the look direction;
- **Y (green) = the frame's LEFT** seen when looking at the sensor **from
  behind**. A wrong Y means the frame is rolled about the look axis even when X
  and Z look plausible.

**Axis colors depend on the framing** — identify axes by LENGTH when in doubt
(Z, up, is ALWAYS the longest arrow): **WITH `object_name`** (selection palette)
Z = light-blue/cyan, X = pink, Y = turquoise; **NO `object_name`** (whole scene)
Z = blue, X = red, Y = green.

To judge the triad, zoom in on the sensor's **parent joint**:
`get_snapshot(object_name=<joint>)` selects it, fits the view onto it, switches
to ISOMETRIC, hides every OTHER joint for the capture (restored right after) and
waits ONE SECOND (Qt events pumped) so the render is up to date. A snapshot with
no argument fits the whole scene and is too far away to see the arrows.

Corrections — rotate the sensor's **parent joint** (never its link) with
`rotate_object(joint_name, axis, angle_deg)`:

- about **X** — tilts the frame forward/back; fixes Z (up). After
  `set_placement_between_sensor_link_and_parent` this is the roll you almost
  always still need: rotate until Z points perpendicularly up (a sideways blue
  arrow means the image is on its side).
- about **Z** — spins the frame left/right; aims a forward-looking sensor or the
  front of a horizontal lidar's sector.
- about **Y** — rolls the frame; fixes Y (left). If the sensor looks the
  OPPOSITE way, verify with `check_overlap` and rotate **180 deg about Y**.

Rotations combine: aim X with a Z rotation, then bring Z up with an X rotation.
Re-check with `get_snapshot(object_name=<joint>)` after each rotation and confirm
ALL THREE axes again before finishing.

**What `set_placement_between_sensor_link_and_parent` does to the frame.** It
snaps the sensor link to the parent link by the CENTERS of the two references and
turns the sensor link's **parent joint** so the sensor's front follows the joint's
local X (red arrow) = the look direction. The ROLL about X is NOT set — that is
the correction you still MUST make (see above).

**Rules for positioning an existing sensor link.** A sensor link is a normal link
that also carries a sensor. Before adding one, CHECK whether it (and its whole
chain from the robot down to it) already exists; if it does, do NOT recreate it —
just POSITION it:

- **Camera / vertical (forward-looking) lidar** →
  `set_placement_between_sensor_link_and_parent(...)` — the ONLY way to connect
  such a sensor link to its parent link; never `set_placement_between`. The roll
  about X must still be corrected.
- **Horizontal lidar** (scans all around) → the EXCEPTION: position it with
  `set_placement_between(...)`, then aim its sector by rotating its **parent
  joint** about **Z** until the sector front (the joint's positive X) points
  where it must.

| Tool | Description |
|---|---|
| `list_sensor_types()` | Available sensors grouped by attachment directory (`link`, `joint`), each with `name`, `type`, `description`. |
| `create_sensor(element, type, name)` | Add a sensor to a link, joint or robot. On a **joint** it needs no position. On a **link** it becomes a child "sensor link" that must already be connected to the parent link by a joint, then positioned (see above). If the sensor link ALREADY exists, do NOT create it again. |
| `set_placement_between_sensor_link_and_parent(sensor, ref_parent, ref_sensor)` | Position a **link**-attached **forward-looking** sensor — the ONLY way to connect such a sensor link to its parent link (see above). `sensor` may be the sensor object **or the sensor link that hosts it** (e.g. `l_camera001`). Joint sensors are rejected; a horizontal lidar uses `set_placement_between` + a Z rotation. |

### Workflow for a link sensor (e.g. a camera, lidar)

Creating the sensor does NOT build its kinematic structure. BEFORE creating it,
CHECK whether the sensor's chain already exists (the sensor link plus every link
and joint from the robot down to it) with `list_scene_objects()` /
`get_objects_info(...)`. If the source objects (bodies / `App::Link`s) are
present, BUILD the chain from them and only THEN attach the sensor; if they are
absent, say so instead of attaching a sensor that has nothing to connect to.

1. (if missing) build the chain: `create_links_filled(robot, [<object>])` (or
   `create_link(robot, ...)` with no geometry to reuse) for the sensor link and
   any intermediate links, connected with
   `create_joint(robot, name, parent_link=..., child_link=...)`. The sensor link
   must be a proper child of the parent link before you continue.
2. `create_sensor('<sensor link>', '<sensor type>')` — skip if it already exists.
3. Position it by its mounting (see "Rules for positioning an existing sensor
   link"): camera / vertical lidar →
   `set_placement_between_sensor_link_and_parent(...)`; horizontal lidar →
   `set_placement_between(...)` + a Z rotation of the parent joint.
4. Verify the frame with `get_snapshot(object_name='<joint>')` and correct it
   until ALL THREE axes (X, Y and Z) are right. Aim until the sensor looks where
   it must: chassis camera forward / lidar all-round, manipulator camera toward
   the end tool.

### Joint sensor

For a joint sensor (e.g. a force-torque sensor) just
`create_sensor('<joint>', 'force_torque')` — no positioning needed.

### Scene and snapshot

| Tool | Description |
|---|---|
| `list_scene_objects()` | JSON description of the scene (robots, links, joints, objects). |
| `get_objects_info(object_names)` | Detailed info about SEVERAL objects in one call (list of names/Labels), merged into `{objects: {...}, errors: {...}}`. |
| `measure_geometry(ref1, ref2, mode)` | Exact measurement in millimetres: distance between two faces/edges/vertices (`ref2` given) or radius of a circular edge / cylindrical face (`ref2` omitted). See "Positioning and rotation". |
| `get_snapshot(width, height, format, object_name)` | 3D view snapshot as an MCP image content block. `object_name` (e.g. a joint) selects it and fits the view onto it so its axes fill the frame; empty = fit the whole scene. Always ISOMETRIC; a **joint** target hides every other joint for the capture (restored right after) and waits one second so the render is up to date. Axis colors depend on the framing — see "Sensors". |

### Agent instructions

| Tool | Description |
|---|---|
| `instructions_to_work_with_tools(topic)` | Step-by-step agent instructions. `topic`: `all` (default, all sections concatenated) or one of `general_algorithm` (source of truth), `create_robot`, `positioning`, `joints`, `collisions`, `materials`, `controllers`, `sensors`, `inspection`, `best_practices`. Does not modify the document. |

### Error reporting (no blocking dialogs)

MCP tools never block on a GUI dialog. The workbench helpers they call
(placement, LCS creation, joint/link proxies, mass calculation, ...) report
problems through `freecad_utils.message` / `warn` / `error` with `gui=True`,
which would otherwise open a modal `QMessageBox` and freeze the call until a
human clicked "OK". While an MCP tool runs, the bridge
(`freecad/cross/mcp/main_thread.py`) suppresses those dialogs (the console
output is kept) and records their text:

- if the tool raises, the recorded text is appended to the exception message —
  the agent receives the real reason of the failure;
- if the tool returns a falsy result (`False` / `None`) together with recorded
  messages, a `RuntimeError` with that text is raised;
- a successful tool returns its result, and any non-fatal message (for example
  a link skipped by `calculate_mass_and_inertia`) is added under `messages`.

Interactive confirmation popups (`ask_confirmation`) and notifications
(`show_notification`) are suppressed as well: no human is available, so
`ask_confirmation` answers "Yes" and continues. When the same functions are
called by a user through the GUI, the dialogs are shown as before.

## Example agent prompt

```
If there is no active document, create a new one (create_document).
Create a robot "arm" with three links (base, link1, link2) and two
revolute joints about the Z axis. Then set the steel material on all
links, compute mass and inertia, and take a 3D view snapshot.
```

## Settings

- The server port is stored in the workbench parameters (`mcp_server_port`,
  default `8006`) and can be changed in the **MCP Agent** dialog.
- The defaults (host, port, endpoint path, env variable names) are defined in
  one place: `freecad/cross/mcp/config.py` — shared by the HTTP server and
  the stdio bridge.
- The default host is `127.0.0.1` (local connections only).

## Installing the dependency

The `mcp` package (the official Model Context Protocol SDK) is installed
automatically, but **lazily**: not at workbench start, but on the first use of
the MCP tools (i.e. the first start of the MCP server, via the
`check_install_package` mechanism like `urdf_parser_py`, `xacro` and others).
This keeps the FreeCAD start-up fast — the first `pip install mcp` may take
some time, but it happens only when the agent functionality is actually used
(McpServer.start()).

> **Important:** the code is compatible with **mcp 2.x** (in 2.x the
> `FastMCP` class is renamed to `MCPServer`, and the low-level `Server`
> accepts the `on_list_tools`/`on_call_tool` handlers in the constructor).
> If version 1.x is installed in your environment, update it:
> `pip install -U mcp`.
