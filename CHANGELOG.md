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
