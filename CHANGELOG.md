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
