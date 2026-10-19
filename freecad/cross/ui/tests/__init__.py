"""Test suite for `inertia_calculator.py`.

Run with a FreeCAD-aware Python interpreter, for example::

    freecadcmd -m unittest discover -s freecad/cross/ui/tests -t .

or, since `unittest.TestCase` subclasses are auto-collected by pytest::

    pytest freecad/cross/ui/tests

(pytest itself must be run with a Python environment that can `import FreeCAD`,
e.g. by pointing `PYTHONPATH` at FreeCAD's `lib`/`Mod` directories, or by
invoking it as `freecadcmd -m pytest ...` if the `pytest` package is
installed into FreeCAD's own interpreter.)

Tests of functions defined in command modules (`import_command()`) and the
selection tests need the FreeCAD GUI (`FreeCADGui`); without it they are
skipped.

Module overview
----------------
base.py
    `InertiaCalculatorTestCase`: creates and tears down a scratch FreeCAD
    document per test, plus numeric assertion helpers for comparing
    generalized 6x6 inertia matrices.
shapes.py
    Helpers that build test geometry (Part primitives, PartDesign
    primitives, compounds, containers, links) in a document.
materials.py
    Helpers that build FreeCAD 1.1 `Materials.Material` objects with a
    known density and assign them to `ShapeMaterial`.
analytic.py
    Independent (not importing `inertia_calculator`) closed-form inertia
    formulas used as expected values: box/cylinder inertia about their
    centroid, and the generalized-6x6-matrix / rigid-placement-transform
    assembly used to combine and relocate them.
test_math_helpers.py
    Pure math/property-reading unit tests for `screw_tf_matrix`,
    `inertia_matrix6d`, `rotation_matrix3`, `get_property` and
    `get_property_optional`.
test_basic_shapes.py
    Case 1: basic Part and PartDesign solids, plus compounds.
test_placement.py
    Case 2: bodies with a non-identity Placement.
test_links.py
    Case 3: `App::Link` to a body, with and without `LinkTransform`.
test_containers.py
    Case 4: `App::Part` and `App::DocumentObjectGroup` as containers.
test_overrides.py
    Case 5: override properties on a plain object, plus validation
    error paths ('manual' mode triangle-inequality/PSD checks, missing
    properties, mode-missing-but-values-present, 'ignore' non-traversal).
test_material_resolution.py
    Material resolution: own material, inherited material, missing
    density, missing material entirely.
test_link_scale.py
    `App::Link` with `ScaleVector` / `Scale` (uniform, non-uniform with
    shear, mirroring, `LinkTransform` on/off) and chains of links with
    placements and scales. References: `analytic.affine_transform_inertia6d`
    and FreeCAD's own shape (`Part.getShape`, see `shapes.shape_inertia6d`).
test_link_element_count.py
    `App::Link` array (`ElementCount` > 0, not a Draft array): four wheels of
    a mobile base, element objects shown or not (`ShowElement`), array
    placement and scale, element scales, a 'mass' override, the array in a
    Part with a chassis; plus the FreeCAD facts the calculator relies on
    (`LinkArrayAssumptionTests`). `LinkTransform = True`:
    `Link.Placement * P_i * B` as displayed by FreeCAD (`Part.getShape()`
    differs and is not used as a reference there).
test_draft_arrays.py
    Draft ortho arrays: link array (with placement, per-element `ScaleList`,
    global `ScaleVector`, `ExpandArray`; material of the base object) and
    regular array (base + `PlacementList`); the base material wins, a
    material on the array is inherited by a base without material.
    Overlapping copies: counted several times without `Fuse`; a fused array
    is one solid with the material of the array.
    References: an analytic model from the link properties of the array
    (checked against `Part.getShape(array)` without scales), and
    `Part.getShape(array)` / `array.Shape` for regular arrays.
test_part_booleans.py
    Part booleans (`Part::MultiFuse` / GUI Union, `Part::Fuse`, `Part::Cut`,
    `Part::Common`) are solids, not containers: the result shape with the
    material of the feature, the inputs and their materials are not used.
test_partdesign_body.py
    `PartDesign::Body` with several features (two additive boxes and a
    pocket): the body is counted once, not once per feature.
test_override_children.py
    Children of a container (or link) in 'density' / 'mass' mode: their
    'manual', 'density' and incomplete override properties are not used,
    'ignore' children are still excluded.
test_logging.py
    Smoke test: both console output levels run to completion and give the
    same numeric result (the output level only affects what is printed).
test_link_integration.py
    Plugin integration helpers: `mass_com_inertia`, `calculate(...,
    ignore_placement=True)` for a link's Real element (local frame, Part
    container as compound), `center_of_mass_placement_in_link_frame` (URDF
    `<inertial>` convention shared with the "basic" method; defined in the
    command module), `log_message` / `log_leader` (tree-shaped console
    log) and the shared enumeration constants.
test_object_inertia.py
    "Calculate object mass and inertia" command: `transform_to_frame` with
    `calculate()` in the local/global frame, `getSubObject(subname, 2)`
    (object and matrix, also with a face/edge/solid at the end of the path),
    the object and global placement of a real selection, and the text output
    (`format_inertial_properties`) of the command module.
test_inertia_frame.py
    "Principal axes" option of the "Calculate object mass and inertia" window:
    `principal_axes()` (ascending moments, right-handed axes, same physical
    inertia) and its text output; `error_summary()` of the "Calculate mass
    and inertia" command.
test_robot_properties.py
    Robot properties `InertiaCalculationMethod` / `InertiaCalculatorLogLevel`:
    defaults, enumerations, migration of robots saved without them.
commands.py
    `import_command()`: imports a command module, skipping the test when the
    GUI is not available (functions defined in command modules).

Upstream issue found while writing these tests (fixed 2026-10-04)
------------------------------------------------------------------
`inertia_matrix6d`'s mass/center-of-mass cross term had its sign
backwards (it computed `J + m*S@S` where the parallel axis theorem
requires `J - m*S@S`). This has since been corrected upstream (now `J -
mS @ S`); see `test_math_helpers.InertiaMatrix6dTests`'s docstring for
the full derivation this was checked against. It was a pre-existing bug,
not one of the two originally found in the 'manual' branch, and affected
every leaf solid `calculate()` computes whenever its center of mass
isn't exactly at its local origin -- the common case. With the fix in
place, the integration tests in test_basic_shapes.py, test_placement.py,
test_links.py, test_containers.py and test_overrides.py should now agree
with `calculate()`'s actual output; they were written against the
physically correct (independently derived) values throughout, so no test
changes were needed to pick up the fix.
"""
