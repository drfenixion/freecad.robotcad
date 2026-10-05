"""Test suite for `inertia_calculator.py`.

Run with a FreeCAD-aware Python interpreter, for example::

    freecadcmd -m unittest discover -s freecad/cross/ui/tests -t .

or, since `unittest.TestCase` subclasses are auto-collected by pytest::

    pytest freecad/cross/ui/tests

(pytest itself must be run with a Python environment that can `import FreeCAD`,
e.g. by pointing `PYTHONPATH` at FreeCAD's `lib`/`Mod` directories, or by
invoking it as `freecadcmd -m pytest ...` if the `pytest` package is
installed into FreeCAD's own interpreter.)

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
