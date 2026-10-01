# MCP Agent (Model Context Protocol)

RobotCAD provides an **MCP server** that lets an external LLM agent
(for example, an agent in VS Code — Roo, Cline, Continue — or Claude Desktop)
control the RobotCAD document: create robots, links, joints and collisions,
position and rotate objects, set materials, compute mass and inertia,
select objects and take 3D view snapshots.

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
| `create_collision(link_or_robot, type)` | Create a collision for a link or robot. **By default call it without `type`** — an exact copy of the geometry (**the default and only allowed collision type**). Primitive types are allowed **only if the user explicitly asks**: `type='box'` (box from the bounding box), `type='sphere'` (sphere from the bounding box), `type='cylinder_x'` / `'cylinder_y'` / `'cylinder_z'` (cylinder from the bounding box, per axis). |

### Positioning and rotation

**The primary positioning method is `set_placement_between`.** After the
joints are created, the contact zones of two neighbouring links are snapped:
one reference (a face, edge, vertex or circle — if its center is needed) on
the parent link and one on the child link, then
`set_placement_between(target, ref1, ref2)` is called (only the two
references go into the selection). A control snapshot is taken
(`get_snapshot`); if a link is oriented incorrectly, the rotation tool
(`rotate_object`) corrects the orientation, after which another control
snapshot is taken to verify. The loop then repeats for the next links until
all links are positioned.

> **If `set_placement_between` returns no error but the target's coordinates
> did NOT change**, you have most likely moved it into its own coordinates
> (the two references resolved to the same point). Verify the target's
> placement with `get_object_info(target)` after the call and pick different
> references (e.g. a face instead of a vertex) if nothing moved.

**Joint axis orientation.** A `revolute` or `continuous` joint **rotates
around its local Z axis** (the blue arrow shown on the joint in the 3D view);
a `prismatic` joint **moves along its local Z axis** (the same blue arrow).
After positioning, the joint can be rotated with the rotation tools
(`rotate_object`) to aim its Z axis in the required direction. Rotating a
joint also rotates its **child kinematic chain and the end link** together
with it, so the whole downstream branch follows the joint orientation — to
orient a wheel/arm correctly, rotate the JOINT, not the link.

**Align the joint's local Z with the child link's functional axis — by
rotating the JOINT, never by rotating the link.** Because a revolute/continuous
joint always spins around its local Z, the child link's functional axis (e.g.
a wheel's axle) must lie along that local Z — otherwise the joint rotates the
link about the wrong axis and the wheel will not roll. The link is mounted on
the joint and follows it, so **rotating the link itself would break this
alignment** (it would turn the link relative to the joint's Z). Instead:

> **Chassis: all wheel Z axes must point the same way (to the left).** For a
> chassis (a parent with several symmetric wheels), all wheels' joint local Z
> axes (the wheel axles) MUST point in the SAME direction — to the LEFT. Do
> not mirror the joints so that opposite wheels point in opposite directions:
> every wheel's Z axis must be parallel and point the same way (left), so that
> all wheels roll consistently. If a wheel's Z axis points the other way,
> rotate its JOINT (not the link) to flip it into the common left direction.

1. **Orient the joint's local Z along the link's functional axis** by rotating
   the JOINT with `rotate_object(joint_name, axis, angle_deg)`. There is no
   `axis` parameter on `create_joint` — the joint's local Z is aimed purely by
   rotating the joint.
2. **Aim the whole assembly by rotating the JOINT** with
   `rotate_object(joint_name, axis, angle_deg)` — this turns the joint's local
   Z (and the child link with it) into the required direction.

Do **not** call `rotate_object` on the link to fix its orientation relative to
the joint: the link must stay aligned with the joint's local Z, and rotating
the link is exactly what breaks that alignment.

**Mirroring a link to the other side of the parent.** There is one case where
rotating the link is the correct fix: when the joint is oriented correctly
**and** the link is oriented correctly, but the link's body penetrates the
parent link's body through its full height (the link should sit on the
opposite side). Rotate the **link** 180° about the **X axis**:

```
rotate_object(link_name, 'x', 180)
```

The link is aligned along its local Z (the functional axis), so rotating it
about X mirrors it about Z: the link flips to the other side of the joint
while its Z-axis alignment and the joint orientation stay intact. Use the
**X axis, not Z**: because the link is aligned along its local Z, rotating it
about Z would only spin the link around its own functional axis and would not
mirror it at all. Use this for wheels on one side, or generally to place
symmetric links on opposite sides of a parent kinematic chain.

> **MANDATORY OVERLAP CHECK — NEVER SKIP, DO IT FOR EVERY CHILD ONE BY ONE.**
> After positioning EACH child link (and after every `rotate_object` on its
> joint) you MUST verify that the child link does NOT overlap the parent
> link's body. Do NOT assume that because one child is fine the others are
> too: symmetric children placed on opposite corners of the same parent
> commonly end up on OPPOSITE sides of the parent's body — some outside
> (correct), some inside (overlapping, wrong). This is exactly the case for a
> 4-wheeled chassis: the wheels on the two corners at one end sit outside,
> while the wheels on the two corners at the other end sit inside the chassis
> body and MUST be mirrored. Procedure (repeat for each child):
> 1. `get_object_info(child_link)` — read the global coordinates of its
>    `Geometry` (`center_of_mass`, `vertices`, face `center_of_mass`).
> 2. `get_object_info(parent_link)` — read the global coordinate range
>    (bounding box) of the parent's `Geometry`.
> 3. If the child's body lies INSIDE the parent's body (its coordinates fall
>    within the parent's bounding range along the axis perpendicular to the
>    mounting face), the child OVERLAPS the parent and MUST be mirrored.
> 4. Mirror it by rotating the LINK 180° about the X axis:
>    `rotate_object(child_link, 'x', 180)`, then re-run `get_object_info` and
>    confirm the child now lies OUTSIDE the parent's body.
>
> A child that overlaps the parent is a positioning FAILURE, not an
> acceptable result. Do NOT proceed to the next child, to collisions, or to
> materials until EVERY child has passed this check.

**Do not create LCS objects** — plain subelement references are enough.
`create_lcs` is used **only if the user explicitly asks for it**.

> `set_object_placement` is currently disabled in the tool registry (the
> implementation is kept in [`tools_registry.py`](../freecad/cross/mcp/tools_registry.py),
> commented out in `TOOLS`). `set_placement_between` replaces it as the
> default positioning workflow.

**A robot link cannot be a reference for `set_placement_between`** — only a
face, edge, vertex, circle or LCS on the Real element of a link. The
subelement reference is given as
`<real_link>.<inner_link_name>.<feature>.<subelement>`, e.g.
`real_l_chassis001_.chassis001.Box.Face3` (`<real_link>` is the Real element
link of the robot link, `real_l_...`; `<inner_link_name>` is the **Name of the
`App::Link` inside the Real element** — e.g. `chassis001`, `wheel001` — **NOT
the name of the source body the link was filled from** — e.g. `chassis`,
`wheel`; `Box` is the feature; `Face3` is the subelement). The path **must**
start with the Real element link (`real_l_...`); a path from the robot link
(`l_...`), e.g. `l_chassis001.chassis001.Box.Vertex3`, is invalid and is
rejected with an error. The path is tolerant: the inner link may be given
either as `Name` or `Label`, intermediate levels may be omitted, and the
internal Real element name (`real_l_...`) is accepted though not required. If
the path cannot be resolved, the error lists the tried variants.

> **Common mistake:** using the source body name (`chassis`, `wheel`) instead
> of the inner link Name (`chassis001`, `wheel001`). The Real element of a
> link contains an `App::Link` whose `Name` is what must appear in the
> reference. Inspect the Real element with `get_object_info('real_l_...')` to
> read the inner link `Name` before building the reference.

| Tool | Description |
|---|---|
| `set_placement_between(target, ref1, ref2, move)` | **Primary positioning method.** Snaps the contact zones of two neighbouring links by two references (analog of `Set Placement - fast`). References: face/edge/vertex/circle of the link Real element as `<real_link>.<inner_link_name>.<feature>.<subelement>` (e.g. `real_l_chassis001_.chassis001.Box.Face3`) or an already existing LCS; a robot link (`l_...`) cannot be a reference — the path must start with the Real element link (`real_l_...`). `<inner_link_name>` is the **Name of the `App::Link` inside the Real element** (e.g. `chassis001`, `wheel001`) — **NOT the source body name** (e.g. `chassis`, `wheel`). Only `ref1` and `ref2` are put into the selection (the `target` is not selected). One reference must lie on the parent link, the other on the child link. **Do NOT create LCS objects** — plain subelement references are enough. `move` defaults to `leaf` — currently only suitable for the final (leaf) element of the kinematic chain. |
| `set_placement_vision_mode(robot)` | Show only the Real elements of the robot links, hide Visual and Collision. Call before positioning. |
| `rotate_object(object_name, axis, angle_deg)` | Rotate the joint Origin / link MountedPlacement / LCS. Used to correct the orientation of the joint and of the link relative to the joint after a control snapshot. |
| `create_lcs(link, subelement)` | Create an LCS on a face/edge/circle/vertex of the link Real element. **Use only if the user explicitly asks for it** — the basic positioning algorithm never requires creating an LCS: use plain subelement references in `set_placement_between` instead. **Important:** the subelement reference is given as `<real_link>.<inner_link_name>.<feature>.<subelement>` (e.g. `real_l_chassis001_.chassis001.Box.Face3`), where `<inner_link_name>` is the **Name of the `App::Link` inside the Real element** (e.g. `chassis001`) — **NOT the source body name** (e.g. `chassis`); a path from the robot link (`l_...`) is invalid. |

### Material and inertia

| Tool | Description |
|---|---|
| `set_material(object_name, material_card_path, density, card_name)` | Set a material on a link/robot (`.FCMat` or density). |
| `calculate_mass_and_inertia(robot_or_link_names)` | Compute mass, inertia and center of mass. |
| `set_joint_values(robot, values)` | Set joint values (degrees/mm). |

### Scene and snapshot

| Tool | Description |
|---|---|
| `list_scene_objects()` | JSON description of the scene (robots, links, joints, objects). |
| `get_object_info(object_name)` | Detailed info about an object, including the contents of its group (`Group`). |
| `get_snapshot(width, height, format)` | 3D view snapshot; returns a base64 `data:` URI for vision agents. |

### Agent instructions

| Tool | Description |
|---|---|
| `instructions_to_work_with_tools(topic)` | Step-by-step instructions for the agent: how to create a robot, position parts and work with the other tools. `topic` selects a section: `all` (default), `general_algorithm` (source of truth), `create_robot`, `positioning`, `joints`, `collisions`, `materials`, `inspection`, `best_practices`. The full text (`all`) is assembled by concatenating the sections. The tool does not modify the document. |

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
