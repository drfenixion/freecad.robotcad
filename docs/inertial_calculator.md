# Advanced inertia calculator

`freecad/cross/ui/inertia_calculator.py` calculates the mass, center of mass
and inertia of a FreeCAD object by walking its tree of sub-objects. It is the
"advanced" inertia calculation method of RobotCAD and is used by:

* the **Calculate mass and inertia** tool, when the robot property
  `InertiaCalculationMethod` is `advanced`: once per link, on the first Real
  element of the link (`link.Real[0]`), in the local frame of that element;
* the **Calculate: mass and inertia of the selected object** tool (dockable
  window), on the selected object, in its local frame or in the global frame.
  Option "Principal axes" (unchecked by default): the inertia matrix is
  expressed in the principal axes of inertia (diagonal matrix, moments in
  ascending order), preceded by the rotation from the target frame axes to
  the principal axes (axis/angle and yaw-pitch-roll). Display only, nothing
  is stored.

This document describes how the walk handles each type of object: detection,
placement, scale and material.

## Result: the generalized mass matrix

`calculate(obj, ...)` returns the 6x6 rigid-body mass matrix

```
JJ = [[J - m S@S,  m S  ],
      [m S.T,      m I3 ]]
```

with `m` the mass, `S = skew(c)` (`c` the center of mass) and `J` the 3x3
inertia matrix about the center of mass. The matrices of several bodies
expressed in the same frame simply add up, and a rigid transform `P` is
applied as `X.T @ JJ @ X` with `X = screw_tf_matrix(P.Matrix.inverse())`.
`mass_com_inertia(JJ)` extracts `(m, c, J)` back.

Units: mm, kg, kg·mm² (FreeCAD's native units; the link properties `Ixx..Izz`
are converted to kg·m² by the tool). In volume mode (see below) the "mass" is
a volume in mm³ (density 1).

Frame of the result: the parent frame of `obj` (its own `Placement` is
applied), or the local frame of `obj` with `ignore_placement=True`.

## Arguments that travel down the tree

| Argument | Meaning |
| --- | --- |
| `calculate_volume` | Volume mode: materials and override modes are not used, only geometry (density 1). Set for the subtree of an object in `density` or `mass` override mode. |
| `ignore_placement` | Do not apply the `Placement` of this object (result in its local frame). |
| `parent_material` | Material inherited from the parent, used when the object has no material of its own. |
| `level`, `output_level` | Log indentation and verbosity. |

## Order of the checks for one object

1. **Override mode.** `RobotCadOverrideMode` is read (missing: `none`; values
   present without the mode: warning).
   * `ignore`: the object and its subtree count for nothing. This also holds
     in volume mode, so `ignore` is the only override that a `density` /
     `mass` parent does not cancel.
2. **Volume mode.** If the parent requested volume mode, the override mode is
   treated as `none` and the override properties are not read at all (a
   child with incomplete override properties does not raise).
3. **Own override** (normal mode only):
   * `density` / `mass`: the value is read, the object and its subtree are
     calculated in volume mode, the override is applied to the result at the
     end of the branch (see below).
   * `manual`: `RobotCadMass`, `RobotCadCenterOfMass` and `RobotCadIxx..Izz`
     are read and checked (positive semi-definite, triangle inequality), the
     matrix is built in the axes of `RobotCadCenterOfMass.Rotation`, moved to
     the object frame, the object `Placement` is applied unless
     `ignore_placement`, and the result is returned: the shape, the material
     and the whole subtree are ignored.
4. **Material.** `ShapeMaterial` of the object unless absent or named
   `Default`; otherwise `parent_material`.
5. **Type detection**, first match wins:

   | Object | Detected by | Processed as |
   | --- | --- | --- |
   | Link (`App::Link`, link element, link-like object) | `getLinkedObject(recursive=False) is not obj` | linked object, link scale and placement |
   | Multi-element `App::Link` (`ElementCount` > 0) | `LinkedObject`, `ElementCount` > 0, `ShowElement` | `LinkedObject` placed by each element (`ElementList` if `ShowElement`, else `PlacementList` / `ScaleList`) |
   | Draft link array, expanded | `Base`, `ExpandArray` = True | `Base` placed by each element of `ElementList` |
   | Draft link array, collapsed | `Base`, `ExpandArray` = False | `Base` placed by `PlacementList` / `ScaleList` |
   | Draft array, not fused | `Base`, `PlacementList`, `Fuse` = False | `Base` placed by `PlacementList` |
   | Container (`App::Part`, Assembly, group...) | non-empty `getSubObjects()`, no `Base` | each child |
   | Solid (incl. fused Draft array, Part booleans `Part::MultiFuse` / `Fuse` / `Cut` / `Common`, `Part::Fillet`, `PartDesign::Body`...) | non-null `Shape` with solids | `Shape.Solids` with the object material; inputs and their materials not used |
   | Anything else | | zero |

   An object with a `Base` property that is not one of the arrays above
   (fused array, boolean, fillet, extrusion...) is never walked as a
   container: it is a solid. Booleans without `Base` (`Part::MultiFuse`,
   created by the GUI Union) have no sub-objects either: their inputs, shown
   under them in the tree, are not counted (`test_part_booleans.py`). Note
   that FreeCAD gives a boolean created without a material the material of
   its inputs (observed for `Part::MultiFuse`); the calculator only reads the
   material of the boolean itself.

## Links

* **Non-recursive on purpose:** in a chain `Link2 -> Link1 -> Body` every
  link is visited and contributes its own placement and scale.
* The linked object is calculated with `ignore_placement = not LinkTransform`
  (`True` when the object has no `LinkTransform` property): with
  `LinkTransform` the placement of the linked object is kept under the link
  placement, without it only the link placement applies.
* The link scale `ScaleVector` (also set by the uniform `Scale`) is applied
  in the frame of the link, before its placement:
  `x_parent = Placement * diag(ScaleVector) * x_linked`. Non-uniform scales
  of rotated geometry (shear) and mirroring scales (negative components,
  positive mass) are handled by `scale_inertia()`, which scales the second
  moments of the mass about the origin (uniform density assumed).
* Then the link `Placement` (unless `ignore_placement`) and the link's own
  `density` / `mass` override.
* The material of the link (if any) is passed to the linked object as parent
  material: the linked object's own material wins.
* A link to an array (`ElementCount` > 0, Draft link array) is not a link
  here: `getLinkedObject()` of such an object returns the object itself.

## Multi-element links and arrays

Links with `ElementCount` > 0 (made with Std_LinkMake, then `ElementCount`
set), Draft link arrays and Draft arrays that are not fused are processed
through their base object (`LinkedObject` or `Base`):

1. **Base.** Calculated **once**, with `ignore_placement = not LinkTransform`
   (`True` for regular Draft arrays, which have no `LinkTransform`: the
   entries of their `PlacementList` already contain the base placement). The
   array material is only passed as parent material: the base keeps its own
   material.
2. **Element transforms**, `x = P_i * diag(S_i) * x_base`, where `x_base`
   includes the base placement `B` if `LinkTransform` is set
   (`Placement * P_i * B`, the order FreeCAD displays):
   * multi-element link with `ShowElement` = True and expanded Draft link
     array: the elements of `ElementList`, their `Placement` and
     `ScaleVector`. For a multi-element link with `ShowElement` = True the
     transforms are only on the elements (`PlacementList` stays empty).
     Draft resets `ScaleList` on expansion; in the tests the elements kept a
     uniform scale equal to the x component of their former `ScaleList`
     entry;
   * multi-element link with `ShowElement` = False and collapsed Draft link
     array: `PlacementList[i]` and `ScaleList[i]`;
   * regular Draft array: `PlacementList[i]` only, no scale.
3. **Global scale** (`ScaleVector` of the object, if any), applied to the sum
   of the elements, before the placement:
   `x = Placement * diag(ScaleVector) * sum_i(P_i * diag(S_i) * x_base)`.
4. Then the object `Placement` (unless `ignore_placement`) and its own
   `density` / `mass` override, as for containers.

Each element is a separate body: **overlapping elements are counted several
times**. Links and link arrays are links to an object, FreeCAD does not fuse
them (their `Fuse` property has no effect), and their scales change the 3D
view but not `Shape.Solids`, so their `Shape` cannot be used instead.

**Fused Draft array** (`Fuse` = True, regular arrays only): processed as a
solid. Its `Shape` is the fused result (overlaps counted once, no individual
scales) and the material of the array (own or inherited) applies to all of
it: the material of the base is not used, and an array without material has
zero mass.

Notes:

* `Part.getShape()` of a multi-element link with `LinkTransform = True`
  applies `B * P_i`, which differs from the 3D view: it is not used as a
  reference (`test_link_element_count.py`).
* `VisibilityList` (elements hidden individually) is not used, as visibility
  in general.

## Containers

Objects with a non-empty `getSubObjects()` (`App::Part`, Assembly,
`App::DocumentObjectGroup`, ...) that are not links or arrays.

* Each child `obj.getSubObject(subname, 1)` is calculated with
  `ignore_placement=False`: child placements are relative to the container.
* Then the container `Placement` (unless `ignore_placement`; groups have
  none) and its own `density` / `mass` override (`mass`: the mass is
  distributed proportionally to the volume of the children).
* The container material is passed down as parent material.
* A `PartDesign::Body` is not a container here (`getSubObjects()` is empty):
  it is a solid with the shape of its Tip, so its features are not counted
  several times.

## Visibility

Visibility has no effect on the calculation: a hidden object inside a
container is counted like any other. Objects that are in a container only
as construction references (e.g. a "master sketch" / skeleton body of an
assembly) must be marked with the `ignore` override mode.

Note: when a Draft array is put into an `App::Part`, its `Base` also becomes
a member of the Part (`Group`), but the tree shows it under the array only.
`getSubObjects()` of the Part does not list it, so the base is counted once,
through the array. In a test script, right after the construction,
`getSubObjects()` was observed to still list the base (counted twice), while
the same call made later from the Python console did not; the cause was not
investigated further. The tests avoid that case (`test_draft_arrays.py`
uses a link with a `density` override instead of a Part).

## Solids

Any object with a non-null `Shape` that has solids (`Solid`, `Compound`,
`CompSolid`, ...): the mass properties of every solid of `Shape.Solids` are
added (OpenCASCADE, density 1).

* `Shape` already contains the object `Placement`: with
  `ignore_placement=True` it is removed again.
* The result is normalized to the volume, then:
  * `none`, volume mode: kept as a volume;
  * `none`: multiplied by the density of the material (own or inherited); no
    material or no `Density` property: zero mass and a warning;
  * `density`: multiplied by `RobotCadDensity`;
  * `mass`: scaled to `RobotCadMass` (uniform density).
* Overlapping solids of a compound are counted twice (no boolean fusion).
* A shape without solids (sketch, wire, shell, datum, `Origin` features...)
  contributes nothing.

## Materials, summary

| Situation | Material used |
| --- | --- |
| Object with its own `ShapeMaterial` (not `Default`) | its own |
| Object without material | the material inherited from the closest ancestor (link, container, array) that has one |
| No material anywhere | zero mass, warning |
| Base of a link array or of a Draft array that is not fused | its own; the array material only if the base has none |
| Fused Draft array | the material of the array (own or inherited), for the whole fused shape |
| Inside a `density` / `mass` override | no material at all: geometry only, the override value applies |
| `manual` override | none: the manual values |

## Override modes, summary

| Mode | Object | Subtree |
| --- | --- | --- |
| `none` | shape and material | processed normally, overrides of children apply |
| `density` | `RobotCadDensity` × volume | geometry only; only `ignore` of children applies |
| `mass` | `RobotCadMass` distributed over the volume | geometry only; only `ignore` of children applies |
| `manual` | `RobotCadMass`, `RobotCadCenterOfMass`, `RobotCadIxx..Izz` | not processed |
| `ignore` | zero | not processed (also inside a `density` / `mass` parent) |

The override properties are added by the **Override inertial properties**
tool (group `RobotCad`); `RobotCadIxx..Izz` are in kg·mm².

## Use by the "Calculate mass and inertia" tool

* The Real element of the link is calculated in its local frame
  (`ignore_placement=True`); `link.MountedPlacement` maps that frame to the
  link frame.
* `CenterOfMass = Placement(MountedPlacement * c, MountedPlacement.Rotation)`
  and `Ixx..Izz` in the axes of the Real element: the URDF
  `<inertial><origin>` and `<inertia>`. Same convention as the basic method.
* Links that cannot be calculated are reported in the Report View and listed
  in one dialog at the end; they keep their previous values, which are used
  in the total mass of the robot.

## Log

`InertiaCalculatorLogLevel` (robot) or the "Log level" of the object window:
`warn` prints only warnings, `info` prints the whole walk as a tree:

```
INFO: Calculate inertial properties of link "arm" (arm), method: advanced.
INFO: └─> object Assembly (Doc#Assembly): container, calculate_volume: False, override_mode: none.
INFO:     └─> object Box1 (Doc#Box1): solids/compound, material: Steel, density: ... g/mm^3, mass: ... g.
```

## Tests

`freecad/cross/ui/tests/` (see its `__init__.py`): analytic references
(`analytic.py`, including a general affine transform for scales) and
FreeCAD's own shapes (`Part.getShape()`) as references. Note:
`Part.getShape()` of a Draft link array or of a multi-element link does not
follow the individual scales, so the tests with scales use an analytic model
of the array instead.
