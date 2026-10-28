# RobotCAD — Release v12.10.5

**Date:** 2026-10-28

## New features

- Added **sensor support to the MCP tools**: `list_sensor_types`, `create_sensor` and `set_placement_between_sensor_link_and_parent`, so an agent can add a sensor to a link, joint or robot and position a forward-looking sensor (camera, vertical lidar) correctly ([`freecad/cross/mcp/tools_registry.py`](freecad/cross/mcp/tools_registry.py)).
- Added **controllers / `ros2_control` support to the MCP tools**: `list_controller_types`, `create_controller`, `get_robot_controllers`, `list_controller_parameters`, `set_controller_parameter` and `set_controller_parameters` ([`freecad/cross/mcp/tools_registry.py`](freecad/cross/mcp/tools_registry.py)).
- Added the **`measure_geometry` MCP tool** — exact measurements in millimetres: the distance between two faces/edges/vertices (real minimum gap, center distance, per-axis delta) or the radius/diameter/axis of a circular edge or cylindrical/spherical/toroidal face. References use the same full subelement path as `set_placement_between`; a bare object name is rejected with a hint naming the correct prefix ([`freecad/cross/mcp/tools_registry.py`](freecad/cross/mcp/tools_registry.py)).

## Improvements

- **`create_collision` → `create_collisions`**: collisions are now created for several links/robots in one call, with either one type for every element or a list parallel to `link_or_robot`. A robot covers all its links; its links passed explicitly are skipped, so nothing is collision-ised twice ([`freecad/cross/mcp/tool_schemas.py`](freecad/cross/mcp/tool_schemas.py)).
- **`get_object_info` → `get_objects_info`**: detailed information about several objects is now returned in one call, with unresolvable names collected under `errors` instead of aborting the request ([`freecad/cross/mcp/tools_registry.py`](freecad/cross/mcp/tools_registry.py)).
- **`get_snapshot`**: added the `object_name` parameter — it selects an object/joint and fits the view onto it, so a joint's axes fill the frame; `get_snapshot` is registered with `structured_output=False` so the image block reaches the agent as an image instead of being re-serialized into truncated text ([`freecad/cross/mcp/server.py`](freecad/cross/mcp/server.py)).
- **Joint snapshots are annotated with `Z` / `X` / `Y` axis letters**, drawn just past the tip of the matching arrow and in the same color, so the labels follow the selection highlight. The letter height is proportional to the joint's `AxisLength` and each label is offset beside its arrow in the image plane (new `text_3d()` helper in [`freecad/cross/coin_utils.py`](freecad/cross/coin_utils.py)).
- The **stdio MCP proxy forwards the remote content blocks unchanged** (instead of flattening them to text), so an image result stays an image ([`freecad/cross/mcp/stdio_server.py`](freecad/cross/mcp/stdio_server.py)).
- Rewritten **MCP agent instructions** ([`docs/mcp_agent.md`](docs/mcp_agent.md)): the sides of a part are defined **as seen from behind** the object, a controller-selection and parameter workflow was added, and geometric values must be obtained with `measure_geometry` instead of estimated from bounding boxes.
- The **"Set placement - sensor"** workflow was extracted into [`set_placement_sensor()`](freecad/cross/wb_utils.py), so the GUI command and the MCP tool share one implementation ([`freecad/cross/ui/command_set_placement_fast_sensor.py`](freecad/cross/ui/command_set_placement_fast_sensor.py)).
- Documented the **JSON shape of `list_controller_parameters` results** in the MCP agent instructions: `current_value` is always JSON — a link/joint parameter reports the object **name** (pass it straight back to `set_controller_parameter`), a vector reports `[x, y, z]`, a placement reports `{x, y, z, q0, q1, q2, q3}` and a quantity reports its plain number ([`docs/mcp_agent.md`](docs/mcp_agent.md)).

## Fixes

- Fixed **MCP tool calls blocking on modal dialogs**: many workbench helpers report problems with a modal `QMessageBox`, which froze the GUI thread (and therefore the tool call waiting for it) until a human clicked "OK". GUI message boxes are now suppressed while a tool runs, `ask_confirmation`/`show_notification` do not block, and the emitted messages are recorded and returned to the agent ([`freecad/cross/mcp/main_thread.py`](freecad/cross/mcp/main_thread.py), [`freecad/cross/freecad_utils.py`](freecad/cross/freecad_utils.py), [`freecad/cross/freecadgui_utils.py`](freecad/cross/freecadgui_utils.py)).
- Fixed the **full traceback printed to the FreeCAD console** when an MCP tool got a wrong object (e.g. `set_placement_between_sensor_link_and_parent`). Anticipated `RuntimeError`/`ValueError` failures are converted to the SDK's `ToolError`, which is logged at INFO without a traceback and hands the real reason to the agent (new [`freecad/cross/mcp/errors.py`](freecad/cross/mcp/errors.py)).
- Fixed the **`PydanticSerializationError: Unable to serialize unknown type: <class 'FeaturePython'>` crash of `list_controller_parameters`**: link/joint parameters (`joints`, `*_wheel_names`, frames, ...) hold `DocumentObject` instances and the raw `getattr()` value was passed straight to the MCP SDK's pydantic serializer. Values are now reduced to JSON — a document object to its `Name`, a vector to `[x, y, z]`, a placement to `{x, y, z, q0..q3}`, a quantity to its plain number — so the reported name can be passed straight back to `set_controller_parameter` ([`freecad/cross/mcp/tools_registry.py`](freecad/cross/mcp/tools_registry.py)).
- Fixed the **same serialization crash for every other MCP tool**: tool results are now passed through one JSON-safe converter in [`get_tools()`](freecad/cross/mcp/tools_registry.py), so a FreeCAD value leaking out of any tool can no longer crash the SDK after the tool has already succeeded (`get_snapshot` is exempt because it returns MCP content blocks) ([`freecad/cross/mcp/tools_registry.py`](freecad/cross/mcp/tools_registry.py)).
- Fixed the **white / blank snapshot of a joint**: a `Cross::Joint` has no `Shape`, so `Std_ViewFitSelection` had an empty bounding box and left the camera where it was. The camera is now fitted to the joint's coin root node and the Qt event loop is pumped before the capture, so the arrows and labels are rendered ([`freecad/cross/mcp/tools_registry.py`](freecad/cross/mcp/tools_registry.py)).
- Fixed the **console warning spam `AttachEngine3D::calculateAttachedPlacement: InertialCS ...`**: [`suppress_console_warnings()`](freecad/cross/freecad_utils.py) silenced only the `Console` observer, while FreeCAD also writes to `ReportOutput` (the GUI "Report view"), `StatusBar` and `NotificationAreaObserver`. The `Wrn` stream of every console observer is now disabled for the block and restored afterwards ([`freecad/cross/freecad_utils.py`](freecad/cross/freecad_utils.py), [`freecad/cross/wb_utils.py`](freecad/cross/wb_utils.py)).
- Fixed the **ROS 2 `none` parameter type** (filter chains such as `sensor_filter_chain` / `filter_chain`), which raised a spurious "Unknown parameter type" warning: it is now mapped to `App::PropertyString`. The misspelled map name `ROS2_CONTROLLERS_PARAMS_TO_FRECAD_PROP_MAP` was renamed to `ROS2_CONTROLLERS_PARAMS_TO_FREECAD_PROP_MAP` ([`freecad/cross/wb_constants.py`](freecad/cross/wb_constants.py), [`freecad/cross/controller_proxy.py`](freecad/cross/controller_proxy.py), [`freecad/cross/sensors/sensor_proxy.py`](freecad/cross/sensors/sensor_proxy.py)).

---

### Commits

- `859255f` — fix `list_controller_parameters` raising an unexpected exception (make tool results JSON-safe, document `current_value` shape)
- `0ae8d64` — bump version
- `1b42e9b` — add measure_geometry mcp tool
- `3159d1e` — add axis labels when snapshop of joint. Fix white screen when snapshot
- `f1f687d` — fix error stacktrace to console when set_placement_between_sensor_link_and_parent get wrong object
- `449f486` — fix console warn spam - "AttachEngine3D::calculateAttachedPlacement:InertialCS"
- `6ad5990` — add sensors and controllers manipulation to mcp tool. Fix get_snapshot mcp tool.

---

# RobotCAD — Release v12.10.4

**Date:** 2026-10-23

## Fixes

- Fixed **joints of a re-imported model losing their parent/child links** when a copy of the same model (e.g. **fingeredu**) already exists in the document. FreeCAD uniquifies the duplicate labels (adding a `001` suffix), while [`set_joint_enum()`](freecad/cross/robot_proxy.py) builds the `Parent` / `Child` enumerations from the link labels but restored the previous value from its `Label2` (URDF) name, which is no longer part of the new enumeration. The value was therefore silently reset to `''` and the joints ended up with no parent/child. The previous value is now resolved to the label that is actually part of the new enumeration before restoring it ([`freecad/cross/robot_proxy.py`](freecad/cross/robot_proxy.py)).
- Fixed the **wrong joint placement and link `MountedPlacement` of the duplicate copy** when the same model (e.g. **fingeredu**) is imported twice into one document (e.g. `finger_upper_to_middle_joint` off its correct position). [`_add_ros_joint()`](freecad/cross/robot_from_urdf.py) stored the raw **URDF** link names into `Parent` / `Child`, while [`_canonicalize_joint_axis()`](freecad/cross/robot_from_urdf.py) and [`_set_child_placement()`](freecad/cross/robot_from_urdf.py) resolve those links through [`ros_name()`](freecad/cross/wb_utils.py) (the label). Because FreeCAD uniquifies the duplicate labels (adding a `001` suffix), the resolution failed and the axis-canonicalization and child-placement compensation silently did nothing, leaving the joints and links of the copy with wrong `Origin` / `MountedPlacement`. The URDF names are now resolved to the labels of the robot's own links before assigning `Parent` / `Child` ([`freecad/cross/robot_from_urdf.py`](freecad/cross/robot_from_urdf.py)).

---

### Commits

- `445cb30` — bump version
- `e833b66` — fix error when import duplicated urdf to same document (same error occurs in models library when import model duplicate to same document)

---

# RobotCAD — Release v12.10.3

**Date:** 2026-10-06

## Fixes

- Fixed the **level-of-detail (Real / Visual / Collision) links of a link falling to the root of the construction tree** after importing a model (e.g. **fingeredu**) or toggling `ShowReal` / `ShowVisual` / `ShowCollision`. FreeCAD does not always rebuild the branch of an `App::Part` after its generated children are deleted and recreated, so the new links were shown at the root even though they were still children of the part. [`refresh_objects_trees()`](freecad/cross/freecadgui_utils.py) re-queries the affected branches (expanding then restoring their state) so FreeCAD shows the links under their actual parent without touching the rest of the tree ([`freecad/cross/link_proxy.py`](freecad/cross/link_proxy.py)).

---

### Commits

- `674c8a2` — bump version
- `7bc1278` — fix lod (Real, Visual, Collision) links subelements dropping to root of building tree

---

# RobotCAD — Release v12.10.2

**Date:** 2026-10-05

## Improvements

- **Dynamic World Generator** (Gazebo map editor, [`modules/Dynamic_World_Generator`](modules/Dynamic_World_Generator)): obstacle and wall colors are now chosen from a **color palette** (`QColorDialog`) instead of typing a color name. The selected color is stored as an exact RGB tuple (`0.0..1.0`), so the canvas preview and the saved SDF always match, and arbitrary palette colors survive a save/load round-trip ([`code/utils/color_button.py`](modules/Dynamic_World_Generator/code/utils/color_button.py), [`code/utils/color_utils.py`](modules/Dynamic_World_Generator/code/utils/color_utils.py), [`code/classes/world_manager.py`](modules/Dynamic_World_Generator/code/classes/world_manager.py)).

## Fixes

- **Dynamic World Generator** — fixed the **silent loss of wall/obstacle changes when Gazebo is installed but not running**: `apply_changes()` skipped writing the SDF whenever the Gazebo service call returned a non-zero code (the supported "generate SDF without a running Gazebo" workflow). Model changes are now always persisted to the SDF file, while runtime service calls are best-effort and gated on a running simulation ([`code/classes/world_manager.py`](modules/Dynamic_World_Generator/code/classes/world_manager.py)).
- **Dynamic World Generator** — fixed **duplicate/overwriting model names**: names were derived from `len(models) + 1`, which collides after removing a model or loading a world (silently overwriting an existing model). Unique names are now generated ([`code/classes/pages/walls_design_page.py`](modules/Dynamic_World_Generator/code/classes/pages/walls_design_page.py), [`code/classes/pages/static_obstacles_page.py`](modules/Dynamic_World_Generator/code/classes/pages/static_obstacles_page.py)).
- **Dynamic World Generator** — fixed an **uncaught `ValueError` when adding a wall** with a non-numeric width/height, and an `AttributeError` in the worlds list refresh before a world/simulation was selected ([`code/classes/pages/walls_design_page.py`](modules/Dynamic_World_Generator/code/classes/pages/walls_design_page.py)).
- Fixed a possible **garbage-collection of the World Generator FreeCAD command**, which could invalidate its QAction and make the menu/toolbar entry unusable; the command instance is now kept in a module-level reference ([`freecad/cross/ui/command_world_generator.py`](freecad/cross/ui/command_world_generator.py)).

---

### Commits

- `b7500f1` — Dynamic_World_Generator: add color picker (palette `QColorDialog` instead of text input; store exact RGB)
- `9f2e208` — Dynamic_World_Generator: fix bugs (always persist model changes to SDF; unique model names; guard wall numeric input; fix worlds list refresh before simulation selection)
- `f62004f` — update Dynamic_World_Generator (fix bugs, add color picker); keep a module-level reference to the `WorldGenerator` FreeCAD command instance
- `f41ea10` — bump version

---

# RobotCAD — Release v12.10.1

**Date:** 2026-10-04

## Fixes

- Fixed a **regression in the MJCF → URDF converter** that tilted some previously-correct joints (e.g. the `FR_calf_joint` / `FL_calf_joint` / `RL_calf_joint` / `RR_calf_joint` of **go1**, and the **spot** knees). Joint limits are now widened to include the home pose **only for joints on a closed kinematic loop** (detected from MuJoCo `connect` / `weld` equality constraints); ordinary joints keep their exact MJCF range and are clamped at home as before ([`freecad/cross/mjcf_urdf_mujoco.py`](freecad/cross/mjcf_urdf_mujoco.py)). This restores the correct tilt of the non-loop joints while keeping the loop joints (e.g. **Cassie** `foot-crank` / `foot`) correct. The converter cache version is bumped to `13`.
- Fixed **URDF import of models whose `<transmission>` blocks have no `<hardwareInterface>`** (e.g. Cassie `cassie_v4.urdf`). `urdf_parser_py` rejects such transmissions with a `ParseError`, which made the Models Library import fail; all `<transmission>` elements are now stripped before parsing ([`freecad/cross/urdf_loader.py`](freecad/cross/urdf_loader.py)).
- Fixed the wording of the Models Library import options: "Don't create solids (quick view only)" and "Remove splitters (edges) from solid`s faces (usefull for Set Placement but increases import time)" ([`freecad/cross/ui/dynamic_ui/models_library.py`](freecad/cross/ui/dynamic_ui/models_library.py)).

---

### Commits

- `6398f0a` — bump version
- `f8814f4` — fix option description
- `f1ca6e4` — fix transmission tag removing when import urdf
- `ced6f8f` — fix mjcf to urdf convertion

---

# RobotCAD — Release v12.10.0

**Date:** 2026-10-03

## New features

- Added a **dependency installation system**: the Python packages required by the workbench are now checked and installed in a unified way (into `~/.local/share/FreeCAD/AdditionalPythonPackages`), with a new dependencies dialog ([`freecad/cross/dependencies.py`](freecad/cross/dependencies.py), [`freecad/cross/ui/dependencies_dialog.py`](freecad/cross/ui/dependencies_dialog.py)).
- Added **MJCF import from the Models Library**: models in MJCF format can be imported directly, using the new MJCF → URDF converter ([`freecad/cross/mjcf_utils.py`](freecad/cross/mjcf_utils.py)).
- Added **xacro import** support for the Models Library.
- Added a **search filter** to the Models Library.
- Updated the **Robots Library** to `robot_descriptions` v3.
- Added a **cloning progress bar** to the Models Library.

## Improvements

- **Performance optimization for URDF import**: `RobotProxy` now has a batch mode that disables loop recalculation of joint Parent/Child and other `onChange` events during import ([`freecad/cross/robot_from_urdf.py`](freecad/cross/robot_from_urdf.py)).
- Added **time metrics** to the URDF importer.
- Models loaded from the library now receive a **semantic name**; the active document is checked/created when the Models Library is used, and adding a model to an empty document is handled correctly.
- `create_without_solids` (remove solid splitter) is now **enabled by default** in the Models Library import; option description fixed.
- Improved **MCP tools**: better `set_placement_between` and rotation usage, geometric spatial perception via `get_object_info`, positioning algorithm added to the agent instructions, `get_snapshot` reactivated, and the `axis` parameter removed from `create_joint`.
- Removed the **PyQt5 dependency** — replaced with FreeCAD's integrated PySide.
- Updated the `ros2_controllers` and `Dynamic_World_Generator` modules.

## Fixes

- Fixed the **MJCF to URDF converter** (joint type orientation and collision generation).
- Fixed the new controllers parameter type (`''` / `none`).

---

### Commits

- `8e03a8c` — set semantic name for models gotten from MJCF and xacro from Models Library; check and create active doc when using Models Library; check adding model from Models Lib to empty doc
- `6e8616a` — add dependencies installation system
- `9c50362` — remove PyQt5 dependency because replaced with FreeCAD's integrated PySide
- `efbff83` — update ros2_controllers module
- `c4dbacb` — add search filter to Models Library
- `1748124` — fix MJCF to URDF converter
- `d761e8b` — fix MJCF to URDF conversion
- `d1c232b` — set create_without_solids as default in Models Library; fix option description
- `cf20dec` — fix Models Library MJCF some joint type orientation
- `c38dfca` — fix Models Library MJCF collision generation
- `ed3458d` — fix Models Library MJCF to URDF collision generation
- `a2e6b33` — fix MJCF import from Models Library
- `d3e81e4` — reactivate get_snapshot MCP tool
- `dc6e1da` — add cloning progressbar to Models Library
- `89130a3` — add MJCF import from Models Library
- `eff159c` — improve MCP tools
- `5768897` — improve MCP tools
- `0d817d4` — improve MCP set_placement_between
- `4e15c81` — improve MCP set_placement_between
- `3b74913` — remove axis param from create_joint
- `9b23be9` — add positioning algorithm to instruction of MCP
- `7c5b323` — fix MCP tools
- `061aa32` — decrease set_placement_between MCP tool description
- `d568bcc` — improve MCP tools
- `2cacb76` — improve MCP Set Placement and Rotation tools usage; add geometric spatial perception to agent by get_object_info
- `2907003` — add xacro import opportunity for Models Library
- `3b03197` — fix new controllers param type - `''`
- `0b2f480` — activate by default remove_solid_splitter option from Models Library import
- `738ff7c` — add time metrics to robot_from_urdf
- `269733d` — fix message
- `a66ea6a` — use batch mode of RobotProxy for URDF import
- `83dac77` — add batch mode to onChange and set_joint_enum in RobotProxy
- `28a6b0e` — update to robot_descriptions v3 (Robots Library)
- `fd9fb87` — update Dynamic_World_Generator; swap PyQt5 to FreeCAD's PySide
- `9fdb2d2` — remove installation of PyQt5
- `a37ff33` — bump version

---

# RobotCAD — Release v12.9.2

**Date:** 2026-09-25

## Fixes

- Fixed the **probability of `Real`, `Visual`, and `Collision` links appearing at the root of the construction tree**: when the group of a `Cross::Link` is reset in `update_fc_links()`, objects that are no longer part of the new group are now removed from the document. Removal is safe if the object was already deleted (e.g. old FreeCAD links removed earlier in the same method), no error is raised in that case.

---

### Commits

- `7eeed89` — fix probability of Real, Visual, and Collision links appearing at the root of the construction tree.
- `4e5ba06` — bump version

---

# RobotCAD — Release v12.9.1

**Date:** 2026-09-24

## Fixes

- Fixed the **orientation of the field-of-view visualization of sensors** (camera frustum and lidar sector): the visualization is now oriented along the **parent joint** of the link the sensor is attached to (X forward, Z up, Y left), not along the link itself, which can be rotated arbitrarily relative to the joint (e.g. by `MountedPlacement`). The parent joint frame is recovered from the link's own properties (`link.Placement * link.MountedPlacement.inverse()`), so the correct orientation is also shown right after loading a document, without depending on the robot structure being fully restored.

---

### Commits

- `22a2217` — fix placement of sensor visualization of field of view
- `aaf2b2b` — fix Set placement - sensor tool description
- `752d166` — fix sensor visualization field of view after file loaded
- `dad705b` — bump version

---

# RobotCAD — Release v12.9.0

**Date:** 2026-09-19

## New features

- Added a **built-in MCP server (Model Context Protocol)** that lets an external LLM agent (for example, Roo, Cline or Continue in VS Code, or Claude Desktop) control the RobotCAD document: create robots, links, joints and collisions, position and rotate objects, set materials, compute mass and inertia, select objects and take 3D view snapshots.
  - **Streamable HTTP transport**: the server runs inside the FreeCAD process in a background thread at `http://127.0.0.1:8006/mcp`, started/stopped from the new **MCP Agent** toolbar button dialog (**Start server**).
  - **stdio transport**: a bridge process [`freecad/cross/mcp/stdio_server.py`](freecad/cross/mcp/stdio_server.py) that connects to the FreeCAD HTTP server and re-exposes the same tools over stdio (for agents that do not support the HTTP transport).
  - **Copy config buttons** in the MCP Agent dialog: **Copy HTTP config** and **Copy stdio config** paste ready-to-use snippets into the agent's MCP settings.
  - 20+ registered tools covering the full robot-building workflow (creation, positioning via `set_placement_between` contact-zone snapping, rotation, LCS, collisions, materials, mass/inertia, joint values, inspection, snapshots). See the tool reference in [`docs/mcp_agent.md`](docs/mcp_agent.md).
  - A workflow guide for the agent (general algorithm, positioning, joints, collisions, materials, inspection, best practices) is exposed via the `instructions_to_work_with_tools` tool.

## Improvements

- **Refactored dependency installation** ([`freecad/cross/packages.py`](freecad/cross/packages.py)): Python packages required by the workbench (including the `mcp` package for the MCP server) are installed into ~/.local/share/FreeCAD/AdditionalPythonPackages in a unified way.

---

### Commits

- `e7421b8` — Add MCP server. Refactor dependies installation.
- `56d031b` — bump version

---

# RobotCAD — Release v12.8.0

**Date:** 2026-09-09

## New features

- Added a **field-of-view (frustum) visualization for camera-type sensors** based on the sensor parameters (`horizontal_fov`, image `width`/`height`, `clip.near`/`clip.far`). The frustum is drawn as a green truncated pyramid in the direction the camera looks at (Gazebo convention: X forward), and is updated when the camera parameters or the sensor placement change.
- Added a **field-of-view visualization for `gpu_lidar` sensors** based on the lidar parameters: a red sector with **80% transparency** spanning the `scan.horizontal` and `scan.vertical` angle ranges between the `range.min` and `range.max` distances (angles/range read by their full parameter path, e.g. `lidar___scan___horizontal___min_angle`).
- The field-of-view visualization of sensors (camera frustum and lidar sector) is now **toggleable with the Space key in both directions** (show and hide), like the robot object: the visualization is drawn into a registered FreeCAD display-mode node, whose visibility is managed by FreeCAD itself (previously the Space key could only show but not hide it).

## Improvements

- **Space-key visibility toggling now works for robot links and joints** (show and hide, both directions): the joint markers (axes and actuation indicators) are drawn into a registered display-mode node, and a display-mode node was added to the link as well, so FreeCAD manages their visibility natively. The visibility of the children (parts, sensors) follows the visibility of their link/joint.

---

### Commits

- `9e8d26c` — add camera type sensors frustum (field of view). Add toggleable visibility (by space button) of frustum (field of view) of camera
- `dece195` — add field of view for gpu_lidar sensor
- `0bf44d2` — make toggleable visibility of robot link and joint by space button
- `822a042` — bump version

---

# RobotCAD — Release v12.7.1

**Date:** 2026-09-05

## Fixes

- Fixed **duplicate `App::Part` wrappers** when binding the same body again via the **New filled robot links** tool (`make_robot_links_filled()`): if a body is already wrapped, its existing `App::Part` wrapper is now **reused** instead of creating a new one. The find-or-create logic is shared between `make_robot_link_filled()` and the manual binding of link elements (`Real`/`Visual`/`Collision`), so the "find existing wrapper" mechanism is not duplicated.

## Improvements

- A robot link created with the **New Link** tool now shows its **Real** geometry by default (`ShowReal = True`), so the view does not have to be toggled before using the **Set Placement** tools for this type of link creation.

---

### Commits

- `9542311` — fix robot link element (visual, real) wrapper recreation when bind same body via make_robot_links_filled()
- `28d99af` — make "ShowReal = True" of new robot link made via "New Link" tool. Let oportunity to not change vision before use Set Placement tools for that type of robot link creation.
- `4c67d5b` — bump version

---

# RobotCAD — Release v12.7.0

**Date:** 2026-09-04

## Improvements

- **Robot link elements bound manually via the FreeCAD Data tab** (`Real`, `Visual`, `Collision` of a Cross::Link) are now **wrapped the same way as in the "filled links" tools**: each manually added object is automatically wrapped into an `App::Part` containing an `App::Link` to the object, the wrapper is hidden and stored into the `robot_parts` container, and the original object is hidden and moved to `robot_parts_origins`.
  - Applies to raw geometry objects **and** to `App::Link` objects that do not point to an `App::Part` (such links are wrapped themselves, preserving their placement and the reference to the linked object).
  - Objects already managed by RobotCAD (`App::Part` wrappers and `App::Link` to a part, `Cross::*` objects) are kept as-is, so programmatic flows (filled-links creation, URDF/KK import, assembly conversion) are **not** double-wrapped.
  - Wrapping is skipped while restoring documents created by older versions, so existing data is preserved when opening a file.

---

### Commits

- `18480da` — add wrapper (Part) for manually added robot link element (real, visual, collision). Manually means via Elements of Data tab of robot link
- `282e6da` — bump version

---

# RobotCAD — Release v12.6.9

**Date:** 2026-08-28

## Fixes

- Fixed the **Assembly → Robot** converter for joints that reference `Part::LocalCoordinateSystem` (LCS) objects: this second type of coordinate system is now detected by `is_lcs()` and handled correctly as a joint reference.

---

### Commits

- `81586f8` — add second type of coordinate system (Part::LocalCoordinateSystem) to CS detection (is_lcs); fix Assembly to RobotCAD conversion with that type of CS as references of joints

---

# RobotCAD — Release v12.6.8

**Date:** 2026-08-26

## Fixes

- A **modal error** is now shown when activating a joint **mimic** without setting the `MimickedJoint` property (previously the error was silenced).

---

### Commits

- `d3ec61b` — set modal error when activated mimic of joint and don't set MimickedJoint (previously silenced error)

---

# RobotCAD — Release v12.6.7

**Date:** 2026-08-17

## Fixes

- Fixed an error in **collision creation** for objects obtained via a link from an **external document**: the temporary collision source object is now removed from its own document (instead of the active one).
- Added **document recomputing** after collision creation, so the created collision objects are properly updated in the model tree.

---

### Commits

- `4a2c970` — fix error in collision creation for objects gotten by link from external document. Add doc recomputing after collision creation

---

# RobotCAD — Release v12.6.6

**Date:** 2026-08-17

## Improvements

- Added the **RobotCAD Display Version** tool — shows the RobotCAD workbench version in a dialog.
- The **Reload Workbench** developer tool is now hidden from the workbench menu.

---

### Commits

- `6672011` — add RobotCAD Display Version tool. Hide Reload Workbench developer tool

---

# RobotCAD — Release v12.6.5

**Date:** 2026-08-16

## Fixes

- Fixed the URDF export for meshes retrieved from an **external document via a link**: the document name is now used as a prefix for the mesh filename, fixing a bug when meshes with the same name came from different external documents.

---

### Commits

- `ce2dd25` — add document prefix for meshes retrieved from an external document via a link

---

# RobotCAD — Release v12.6.4

**Date:** 2026-08-15

## Improvements

- **Set Placement** tools now support `Part::LocalCoordinateSystem` (LCS) objects, not only `PartDesign::CoordinateSystem`.
- **Set Placement** tools now work correctly with links to parts/bodies from **external documents**.
- LCS references from external documents are now handled correctly by **Set Placement** tools.
- The **Set Placement as group** tool now allows selecting **2 LCS in the same robot link** (including grouped selection).
- Added a clear error message when trying to convert a FreeCAD Assembly without any joint.

## Fixes

- Fixed the **Assembly → Robot** converter: previously, an assembly with only one joint did not create a robot joint — conversion now works correctly.
- Fixed `get_child_joints()` in `wb_utils.py` — child joint lookup now uses the correct link name (rare error case).
- Fixed the **Set Placement with hold downstream chain** tool.

---

### Commits

- `55647e3` — let work Set Placement tools with links to external document part/bodies
- `8461986` — let lcs tool references from external docs works with Set Placement tools
- `c5c8f1a` — support `Part::LocalCoordinateSystem` for Set Placement tools
- `a457d0c` — fix Set Placement with hold downstream chain tool
- `bbaeab8` — fix `get_child_joints()` in `wb_utils.py`
- `a8da16a` — let Set Placement as group use 2 LCS in same robot link
- `fcaf439` — fix Assembly to Robot converter in case with only 1 joint
- `63ef0ae` — add error message when trying to Convert Assembly without any joint
