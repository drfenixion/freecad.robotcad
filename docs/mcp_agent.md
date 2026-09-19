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
| `create_joint(robot, name, parent_link, child_link, type, axis, lower, upper, effort, velocity)` | Create a joint between two links and set type/axis/limits. |
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

**Do not create LCS objects** — plain subelement references are enough.
`create_lcs` is used **only if the user explicitly asks for it**.

> `set_object_placement` is currently disabled in the tool registry (the
> implementation is kept in [`tools_registry.py`](../freecad/cross/mcp/tools_registry.py),
> commented out in `TOOLS`). `set_placement_between` replaces it as the
> default positioning workflow.

**A robot link cannot be a reference for `set_placement_between`** — only a
face, edge, vertex, circle or LCS on the Real element of a link. The
subelement reference is given as `<real_link>.<body>.<subelement>`, e.g.
`real_l_chassis001_.chassis001.Box.Face3` (`<real_link>` is the Real element
link of the robot link, `real_l_...`; `<body>` is the body inside the link
Real element; `Face3` is the subelement). The path **must** start with the
Real element link (`real_l_...`); a path from the robot link (`l_...`), e.g.
`l_chassis001.chassis001.Box.Vertex3`, is invalid and is rejected with an
error. The path is tolerant: the body may be given either as `Name` or
`Label`, intermediate levels may be omitted, and the internal Real element
name (`real_l_...`) is accepted though not required. If the path cannot be
resolved, the error lists the tried variants.

| Tool | Description |
|---|---|
| `set_placement_between(target, ref1, ref2, move)` | **Primary positioning method.** Snaps the contact zones of two neighbouring links by two references (analog of `Set Placement - fast`). References: face/edge/vertex/circle of the link Real element as `<real_link>.<body>.<subelement>` (e.g. `real_l_chassis001_.chassis001.Box.Face3`) or an already existing LCS; a robot link (`l_...`) cannot be a reference — the path must start with the Real element link (`real_l_...`). Only `ref1` and `ref2` are put into the selection (the `target` is not selected). One reference must lie on the parent link, the other on the child link. **Do NOT create LCS objects** — plain subelement references are enough. `move` defaults to `leaf` — currently only suitable for the final (leaf) element of the kinematic chain. |
| `set_placement_vision_mode(robot)` | Show only the Real elements of the robot links, hide Visual and Collision. Call before positioning. |
| `rotate_object(object_name, axis, angle_deg)` | Rotate the joint Origin / link MountedPlacement / LCS. Used to correct the orientation of the joint and of the link relative to the joint after a control snapshot. |
| `create_lcs(link, subelement)` | Create an LCS on a face/edge/circle/vertex of the link Real element. **Use only if the user explicitly asks for it** — the basic positioning algorithm never requires creating an LCS: use plain subelement references in `set_placement_between` instead. **Important:** the subelement reference is given as `<real_link>.<body>.<subelement>` (e.g. `real_l_chassis001_.chassis001.Box.Face3`); a path from the robot link (`l_...`) is invalid. |

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
| `get_snapshot(path, width, height, format)` | 3D view snapshot; returns the file path and a base64 `data:` URI for vision agents. |

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
