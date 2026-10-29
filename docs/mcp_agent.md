# MCP Agent (Model Context Protocol)

RobotCAD provides an **MCP server** that lets an external LLM agent (Roo, Cline,
Continue, Claude Desktop, ...) control the RobotCAD document: create robots,
links, joints and collisions, position and rotate objects, set materials,
compute mass/inertia, measure geometry and take 3D view snapshots.

## Quick start

1. Open FreeCAD with the RobotCAD workbench loaded.
2. Toolbar → **MCP Agent** (robot-with-connector icon) → **Start server**.
   An HTTP MCP server starts on `http://127.0.0.1:8006/mcp`.
3. Copy the config (**Copy HTTP config** / **Copy stdio config**) into your
   agent's settings.

### VS Code (`.mcp.json`)

> Roo / Cline expect `mcpServers` as an **object** and the HTTP transport type
> `streamable-http` (not `http`, which is rejected).

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

### stdio (agents without HTTP transport)

```json
{
  "mcpServers": {
    "robotcad": {
      "type": "stdio",
      "command": "<python-exe>",
      "args": ["<path-to-workbench>/freecad/cross/mcp/stdio_server.py"],
      "env": { "MCP_FREECAD_URL": "http://127.0.0.1:8006/mcp" }
    }
  }
}
```

The path can be copied from the **MCP Agent** dialog. FreeCAD must be running
with the HTTP server started while the agent uses the tools.

## Transports

| Transport | Description |
|---|---|
| **Streamable HTTP** | Server inside the FreeCAD process, background thread; agents connect to `http://127.0.0.1:8006/mcp`. |
| **stdio** | Bridge process `freecad/cross/mcp/stdio_server.py` re-exposing the same tools over stdio. |

## Conventions

All tools operate on the **active document** and accept objects by `Name` or
`Label`. **Prefer `Label`** (human-readable, e.g. `left wheel`) over the internal
`Name` (e.g. `l_wheel001_`): pass it to every object argument and use it in
everything reported to the user; fall back to `Name` only when the Label is
missing or ambiguous (`Ambiguous label "..."` lists the candidates).

Subelement references are Name-based and have the form
`<real_link>.<inner_link_name>.<feature>.<subelement>`, e.g.
`real_l_chassis001_.chassis001.Box.Face3`. Build them from `Reference.prefix` of
`get_objects_info([link])` and append `.FaceN`/`.VertexN`/`.EdgeN` — never guess
the inner link name (links from one source body share it: all four wheels →
`wheel001`). A robot link (`l_...`) is never a reference.

The full workflow, the source of truth, is the `general_algorithm` topic of
[`instructions_to_work_with_tools`](../freecad/cross/mcp/tools_registry.py).
Write it as a TODO list and work through it in order; the sections below only add
tool reference detail.

## Tools

### Creation

| Tool | Description |
|---|---|
| `create_document(name)` | New FreeCAD document, made active. |
| `create_robot(name)` | Empty `Cross::Robot`. |
| `create_link(robot, name, add_to_robot)` | Empty `Cross::Link`, optionally added to the robot. |
| `create_links_filled(robot, object_labels)` | Links filled with Real/Visual from existing objects (given by `Label`), in the given order. |
| `create_joint(robot, name, parent_link, child_link, type, lower, upper, effort, velocity)` | Joint between two links, with type and limits (deg/mm). `name` applies ONLY to a joint without links; with both links the joint is named automatically. No `axis` parameter — orient local Z with `rotate_object`. |
| `create_joints_filled(robot, link_labels_in_order, connect_type)` | Joints over the given link Labels: `chain` (consecutive) or `spider` (all to the first). |

### Joints, materials, inertia

| Tool | Description |
|---|---|
| `set_joint_values(robot, values)` | Joint ROS names (or Labels) → degrees (revolute) / mm (prismatic). |
| `set_material(object_name, material_card_path, density, card_name)` | `.FCMat` path or material name; density from the card if omitted. |
| `calculate_mass_and_inertia(robot_or_link_labels)` | Mass, inertia, center of mass (robot Label or link Labels). |

A joint rotates around its **local Z** (revolute/continuous) or translates along
it (prismatic); orient that Z by rotating the **joint**, never the link.

### Positioning and measurement

| Tool | Description |
|---|---|
| `set_placement_between(target, ref1, ref2, move)` | **Primary positioning method**. One reference on the parent, one on the child, joined **by their centers**. Only the two references are selected. `move='leaf'` (default, final chain element) / `child_branch` / `parent_tree`. Never on a camera-like sensor link; a link with only a **horizontal lidar** IS positioned here, then its parent joint is rotated about Z. |
| `set_placement_between_sensor_link_and_parent(sensor, ref_parent, ref_sensor)` | **The only way** to connect a link-attached forward-looking sensor (camera, vertical lidar). Joins the references by their centers and turns the parent joint so the sensor looks along its local X. The roll about X must still be corrected. `sensor` may be the sensor or its sensor link. |
| `set_placement_vision_mode(robot)` | Show only Real elements, hide Visual/Collision. Call before positioning. |
| `rotate_object(object_name, axis, angle_deg)` | Rotate a joint `Origin`, link `MountedPlacement` or LCS. Mirror a link across its parent with the LINK 180° about **X** (never Z — Z only spins it). |
| `create_lcs(link, subelement)` | LCS on a Real-element subelement. **Only on explicit request** — plain references suffice. |
| `measure_geometry(ref1, ref2, mode)` | Exact measurement in **millimetres**. With `ref2`: `distance` (real minimum gap), `center_distance`, per-axis `delta`, `closest_points`. Without `ref2`: `radius`/`diameter`/`center`/`axis` of a circular edge or a cylindrical/spherical/toroidal/conical face. Full subelement paths or LCS only. Use it for every value that must be exact — never estimate from bounding boxes. |
| `check_overlap(parent, child)` | Deterministic boolean `overlap` (a shared face is not an overlap), plus per-axis AABB intervals and `min_distance`. |

**Overlap check** is mandatory per child, after its joint is oriented and before
collisions/materials; on failure mirror the child and re-check. **Sides** are
defined as seen from BEHIND the object, looking along its forward direction.

### Collisions

| Tool | Description |
|---|---|
| `create_collisions(link_or_robot, type)` | Several links/robots in ONE call. `type` is one type for all or a list parallel to `link_or_robot`. **Call without `type` by default** (`copy` = exact geometry). Primitives (`box`, `sphere`, `cylinder_x/y/z`, from the bounding box) only on explicit request. A robot covers all its links; its own links passed in the same call are dropped first. |

### Controllers and broadcasters (ros2_control)

Choose them from the robot's **purpose** and its **actuated joints**:

- Actuated joints → `joint_state_broadcaster` (`kind='broadcaster'`).
- Chassis → a drive controller for its wheels (`diff_drive_controller`,
  `ackermann_steering_controller`, an omni controller, ...) with its
  `*_wheel_names` / `steering_*_names` / frame parameters.
- Manipulator → `joint_trajectory_controller`; gripper →
  `gripper_action_controller` or a position/trajectory controller; other groups
  (pan-tilt, lift, ...) → a suitable trajectory/velocity/position controller.
- **Wheel separation** = distance between the **nearest facing inner faces** of
  the left and right wheel; **wheel radius** from the rim edge or cylindrical
  face. Measure both with `measure_geometry`.
- Distance parameters are in **metres** unless their `description` says otherwise
  — convert the millimetre measurement first.

| Tool | Description |
|---|---|
| `list_controller_types()` | Available controllers and broadcasters (`name`, `description`, `plugin_class_name`, `parameter_count`). |
| `create_controller(robot, type, name, kind)` | Add a controller or broadcaster. `type` = ROS controller name or plugin class; `kind` = `'controller'` (default) / `'broadcaster'`; `name` overrides the Label. |
| `get_robot_controllers(robot)` | Controllers and broadcasters on a robot. |
| `list_controller_parameters(controller)` | `full_name`, `type_fc`, `type`, `default_value`, `description`, `current_value`. A link/joint `current_value` reports the object name, a vector `[x, y, z]`, a placement `{x, y, z, q0..q3}`, a quantity its number. |
| `set_controller_parameter(controller, parameter, value)` | One parameter by full name; nested names joined with `'___'`. Joint/link parameters accept a name or a list of names. |
| `set_controller_parameters(controller, values)` | Several parameters at once. |

Typical flow: robot with links/joints → identify functional joint groups →
`joint_state_broadcaster` → per group: `list_controller_types()`,
`create_controller(...)`, `list_controller_parameters(...)`,
`set_controller_parameter(...)`.

### Sensors

A robot hosts `Cross::Sensor` objects (Gazebo sensors from `resources/sensors`)
attached to a `Cross::Link`, `Cross::Joint` or `Cross::Robot`. A sensor has **no
position of its own**: it is oriented by the parent joint of its element. Attach
each sensor where it **belongs by meaning** (camera on the head/end tool, lidar on
the chassis, encoder on the wheel joint, IMU on the base) — never the root link by
default.

Choose the set (mutually exclusive, in this order):

1. What the user asked for.
2. Sensors already present in the source objects.
3. The **default set**: RGB-D camera (`rgbd_camera`) looking FORWARD plus a 360°
   lidar (`gpu_lidar`) on the chassis, and an RGB-D camera on the manipulator's
   end link aimed at the end tool (skip if no manipulator; no chassis → put them
   on the base link).

**Frame check (X = look, Y = left, Z = up)** — mandatory. Verify the sensor's
parent joint by the `X`/`Y`/`Z` letters next to the arrows of
`get_snapshot('<joint>')`: X = look direction; Z = up **relative to the parent
part**; Y = the frame's LEFT seen from behind (a wrong Y means the frame is
rolled). Correct by rotating the **parent joint**: X tilts (sets Z up), Z spins
(aims the sensor or lidar sector), Y rolls (sets Y left; 180° about Y if the
sensor looks the opposite way).

**Positioning an existing sensor link** (check first whether it and its whole
chain exist — do not recreate):

- Camera / vertical lidar → `set_placement_between_sensor_link_and_parent`.
- Horizontal lidar → `set_placement_between`, then rotate the parent joint about Z.

Creating a sensor does not build its kinematic structure: build the chain first
(`create_links_filled` / `create_link` + `create_joint`) from the existing source
objects.

| Tool | Description |
|---|---|
| `list_sensor_types()` | Sensors grouped by attachment directory (`link`, `joint`). |
| `create_sensor(element, type, name)` | Sensor on a link, joint or robot. On a joint it needs no position; on a link it becomes a child sensor link already connected by a joint. |
| `set_placement_between_sensor_link_and_parent(sensor, ref_parent, ref_sensor)` | See above. Joint sensors are rejected; a horizontal lidar uses `set_placement_between` + a Z rotation. |

### Scene and snapshot

| Tool | Description |
|---|---|
| `list_scene_objects()` | JSON description of the scene (robots, links, joints, objects). |
| `get_objects_info(object_labels)` | Details for SEVERAL objects in one call (list of Labels), merged into `{objects: {...}, errors: {...}}`. Each entry has basic info plus `Geometry` (global faces/vertices, `center_of_mass`, `surface_type`) and a `Reference` block with the exact `prefix` for subelement paths. |
| `measure_geometry(ref1, ref2, mode)` | See "Positioning and measurement". |
| `get_snapshot(width, height, format, object_name, view)` | 3D view as an MCP image block. `object_name` (e.g. a joint) selects it and fits the view onto it so its axes fill the frame; empty = whole scene. `view` = camera side: `isometric` (default), `axonometric`, `top`, `bottom`, `front`, `rear`, `left`, `right` (aliases `iso`, `axo`, `back`, `side`, `default`) — call it once per side; the framing is kept and only the viewpoint turns. Sensor field-of-view zones are hidden and a joint target also hides every other joint (both restored after), waiting one second. A joint's axes are labelled `X`/`Y`/`Z`. |
| `check_overlap(parent, child)` | See "Positioning and measurement". |

### Agent instructions

| Tool | Description |
|---|---|
| `instructions_to_work_with_tools(topic)` | Step-by-step agent instructions. `topic`: `all` (default) or one of `general_algorithm` (source of truth), `create_robot`, `positioning`, `joints`, `collisions`, `materials`, `controllers`, `sensors`, `inspection`, `best_practices`. Does not modify the document. |

## Error reporting (no blocking dialogs)

MCP tools never block on a GUI dialog. While a tool runs, the bridge
([`main_thread.py`](../freecad/cross/mcp/main_thread.py)) suppresses the
workbench's `QMessageBox` popups (console output is kept) and records their text:

- a raised error carries the recorded text, so the agent sees the real reason;
- a falsy result with recorded messages raises a `RuntimeError` with that text;
- a successful tool adds non-fatal messages under `messages`.

Interactive confirmations are suppressed too (no human is available, so
`ask_confirmation` answers "Yes"). Called from the GUI, the dialogs appear as
before.

## Example agent prompt

```
If there is no active document, create a new one (create_document).
Create a robot "arm" with three links (base, link1, link2) and two
revolute joints about the Z axis. Then set the steel material on all
links, compute mass and inertia, and take a 3D view snapshot.
```

## Settings and dependency

- The server port is stored in the workbench parameters (`mcp_server_port`,
  default `8006`) and editable in the **MCP Agent** dialog; the host defaults to
  `127.0.0.1`. Defaults (host, port, endpoint, env var names) live in
  [`config.py`](../freecad/cross/mcp/config.py), shared by the HTTP server and
  the stdio bridge.
- The `mcp` package is installed **lazily** — on the first start of the MCP
  server, not at workbench start — keeping FreeCAD start-up fast.
- Requires **mcp 2.x** (in 2.x `FastMCP` is renamed to `MCPServer`). Update with
  `pip install -U mcp` if 1.x is installed.
