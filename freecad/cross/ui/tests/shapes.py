"""Geometry builders used by the integration tests.

Every function here returns the `DocumentObject` that should be passed to
`inertia_calculator.calculate()`, after a `doc.recompute()` has already
made its `Shape` valid.
"""
from __future__ import annotations

from typing import Iterable, Optional, Sequence

import numpy as np

import FreeCAD as fc

from . import analytic


def make_part_box(doc: fc.Document, name: str, length: float, width: float, height: float, placement: Optional[fc.Placement] = None) -> fc.DocumentObject:
    """Create a `Part::Box` (Part workbench primitive)."""
    box = doc.addObject('Part::Box', name)
    box.Length = length
    box.Width = width
    box.Height = height
    if placement is not None:
        box.Placement = placement
    doc.recompute()
    return box


def make_part_cylinder(doc: fc.Document, name: str, radius: float, height: float, placement: Optional[fc.Placement] = None) -> fc.DocumentObject:
    """Create a `Part::Cylinder` (Part workbench primitive)."""
    cyl = doc.addObject('Part::Cylinder', name)
    cyl.Radius = radius
    cyl.Height = height
    if placement is not None:
        cyl.Placement = placement
    doc.recompute()
    return cyl


def make_partdesign_box(doc: fc.Document, name: str, length: float, width: float, height: float, placement: Optional[fc.Placement] = None) -> fc.DocumentObject:
    """Create a `PartDesign::Body` whose shape is a single `PartDesign::AdditiveBox`.

    `AdditiveBox`/`AdditiveCylinder` are used instead of a Sketch+Pad so
    the geometry can be built with plain property assignment, with no
    sketcher scripting involved. The returned object is the Body (its
    `.Shape` is the box's shape); the Body, not the feature, carries the
    `Placement` that positions the whole part in the document, matching
    how a PartDesign body is normally placed.
    """
    body = doc.addObject('PartDesign::Body', name)
    box = doc.addObject('PartDesign::AdditiveBox', f'{name}_Box')
    box.Length = length
    box.Width = width
    box.Height = height
    body.Group = [box]
    body.Tip = box
    if placement is not None:
        body.Placement = placement
    doc.recompute()
    return body


def make_partdesign_cylinder(doc: fc.Document, name: str, radius: float, height: float, placement: Optional[fc.Placement] = None) -> fc.DocumentObject:
    """Create a `PartDesign::Body` whose shape is a single `PartDesign::AdditiveCylinder`."""
    body = doc.addObject('PartDesign::Body', name)
    cyl = doc.addObject('PartDesign::AdditiveCylinder', f'{name}_Cylinder')
    cyl.Radius = radius
    cyl.Height = height
    body.Group = [cyl]
    body.Tip = cyl
    if placement is not None:
        body.Placement = placement
    doc.recompute()
    return body


def make_compound(doc: fc.Document, name: str, children: Sequence[fc.DocumentObject], placement: Optional[fc.Placement] = None) -> fc.DocumentObject:
    """Create a `Part::Compound` joining `children`'s shapes into one `Compound` shape.

    Note this produces a solid-with-a-Shape object whose `Shape.ShapeType`
    is `'Compound'` (handled by `calculate()`'s "solid objects" branch,
    which sums over `shape.Solids`); it is not a container in the
    `getSubObjects()` sense exercised by `test_containers.py`.
    """
    compound = doc.addObject('Part::Compound', name)
    compound.Links = list(children)
    if placement is not None:
        compound.Placement = placement
    doc.recompute()
    return compound


def make_part_container(doc: fc.Document, name: str, children: Iterable[fc.DocumentObject], placement: Optional[fc.Placement] = None) -> fc.DocumentObject:
    """Create an `App::Part` containing `children`.

    Each child keeps whatever `Placement` it already has; `calculate()`'s
    container branch treats that as being expressed in the container's own
    local frame and composes the container's own `Placement` on top of the
    summed children (see the module docstring in test_containers.py for
    why this, rather than some other interpretation of nested placements,
    is the behavior under test).
    """
    part = doc.addObject('App::Part', name)
    for child in children:
        part.addObject(child)
    if placement is not None:
        part.Placement = placement
    doc.recompute()
    return part


def make_group(doc: fc.Document, name: str, children: Iterable[fc.DocumentObject]) -> fc.DocumentObject:
    """Create an `App::DocumentObjectGroup` containing `children`.

    `App::DocumentObjectGroup` has no `Placement` property of its own, so
    unlike `make_part_container` there is no container-level transform to
    apply; `calculate()`'s container branch falls through the
    `hasattr(obj, 'Placement')` check for it.
    """
    group = doc.addObject('App::DocumentObjectGroup', name)
    for child in children:
        group.addObject(child)
    doc.recompute()
    return group


def make_link(doc: fc.Document, name: str, linked_obj: fc.DocumentObject, placement: Optional[fc.Placement] = None, link_transform: bool = True) -> fc.DocumentObject:
    """Create an `App::Link` pointing at `linked_obj`."""
    link = doc.addObject('App::Link', name)
    link.LinkedObject = linked_obj
    link.LinkTransform = link_transform
    if placement is not None:
        link.Placement = placement
    doc.recompute()
    return link


def shape_inertia6d(shape, density: float) -> np.ndarray:
    """Generalized 6x6 matrix of the solids of a Part shape with a uniform `density` (kg/mm^3).

    Reference built from FreeCAD's own geometry (`Part.getShape(obj)` gives
    the shape of an object with all link placements, scales and array
    elements applied), so it captures FreeCAD's transform semantics
    independently of `inertia_calculator.py`. Each solid's
    `MatrixOfInertia` and `CenterOfMass` are computed by OpenCASCADE with
    density 1 (mass == volume).
    """
    JJ = np.zeros((6, 6))
    for solid in shape.Solids:
        M = solid.MatrixOfInertia
        J = np.array([
            [M.A11, M.A12, M.A13],
            [M.A21, M.A22, M.A23],
            [M.A31, M.A32, M.A33],
        ])
        JJ += analytic.generalized_inertia6d(density * J, solid.CenterOfMass, density * solid.Volume)
    return JJ
