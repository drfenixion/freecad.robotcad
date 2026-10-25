"""Regression tests for "PropertyLinkSubList does not support external object".

Reported traceback (tools "NewLCSAtRobotLinkBody", "SetCROSSPlacementFast")::

    wb_utils.set_placement_fast()
      -> set_placement_by_orienteer()
        -> get_placement_of_orienteer()
          -> make_lcs_at_link_body()
             setattr(lcs, lcs_attachmentsupport_name(), [(original_obj, sub_element_name)])
    ValueError: PropertyLinkSubList does not support external object

Minimal structure built by the tests (the same structure RobotCAD builds for
the Real element of a robot link, see `link_proxy._make_robot_link_element_wrapper()`
and `link_proxy.make_lod_links()`)::

    Robot document                         Geometry document (external file)
    --------------                         ---------------------------------
    real_link_  (App::Link, "dynamic link", Name starts with 'real_')
      -> Part   (App::Part, local wrapper)
           Link (App::Link)  ------------>  Assembly (App::Part)
                                              Box (Part::Box)  <- Face6 selected

The selection is `real_link_` + sub-element name `Link.Box.Face6`, i.e. what
`FreeCADGui.Selection.getSelectionEx('', 0)` returns when the user clicks the
top face of the box in the 3D view.

The old `make_lcs_at_link_body()` attached the temporary LCS (inside the local
wrapper `Part`) to the object found *inside the Real geometry* (`Box` of the
geometry document). `AttachmentSupport` is an `App::PropertyLinkSubList`,
which refuses objects of another document, hence the error. In the same
document it did not raise, but the LCS kept a reference into the Real
geometry and the placement of the wrapper's App::Link was lost.

According to the RobotCAD design the LCS must not reference objects inside
the Real geometry: only the placement of the selected sub-element relative to
the wrapper `Part` is computed and stored in `LCS.Placement`.

Expected values are computed independently of the code under test:
`real.getSubObject(sub, transform=False)` returns the selected sub-element
expressed in the frame of the content of `real_link_`, i.e. in the frame of
the wrapper `Part` (the dynamic link replaces the `Part` placement), which is
the frame `LCS.Placement` is expressed in (LCS -> Body (identity) -> Part).

Only the position of the LCS is checked: its orientation depends on the
attacher mode (InertialCS for a face) and on the "align Z axis" preference.

Tests run without the GUI: a stand-in for `Gui::SelectionObject` is used,
`make_lcs_at_link_body()` only reads `Object`, `SubElementNames`,
`SubObjects` and calls `isDerivedFrom('Gui::SelectionObject')`.
"""
from __future__ import annotations

import shutil
import tempfile
import unittest
from pathlib import Path

import FreeCAD as fc

from .. import wb_utils


SUB_ELEMENT = 'Link.Box.Face6'  # Top face of the box, through the wrapper's App::Link.

# Non-identity placements everywhere, so that a forgotten or doubled
# transformation is visible in the LCS position.
ASSEMBLY_PLACEMENT = fc.Placement(fc.Vector(10.0, 20.0, 30.0), fc.Rotation())
BOX_PLACEMENT = fc.Placement(fc.Vector(1.0, 2.0, 3.0), fc.Rotation())
WRAPPER_PART_PLACEMENT = fc.Placement(fc.Vector(100.0, 0.0, 0.0), fc.Rotation())
LINK_IN_WRAPPER_PLACEMENT = fc.Placement(fc.Vector(0.0, 0.0, 500.0), fc.Rotation())


class FakeSelectionObject:
    """Minimal stand-in of `Gui::SelectionObject` (not constructible from Python)."""

    def __init__(self, obj: fc.DocumentObject, sub_element_name: str) -> None:
        self.Object = obj
        self.ObjectName = obj.Name
        self.Document = obj.Document
        self.DocumentName = obj.Document.Name
        self.SubElementNames = (sub_element_name,)
        self.SubObjects = (obj.getSubObject(sub_element_name),)
        self.HasSubObjects = True

    def isDerivedFrom(self, type_id: str) -> bool:  # noqa: N802 (FreeCAD API name)
        return type_id == 'Gui::SelectionObject'


class MakeLcsAtLinkBodyTests(unittest.TestCase):

    def setUp(self) -> None:
        self._tmp_dir = Path(tempfile.mkdtemp(prefix='robotcad_lcs_test_'))
        self._doc_names: list[str] = []

    def tearDown(self) -> None:
        for name in reversed(self._doc_names):
            try:
                fc.closeDocument(name)
            except Exception:
                pass
        shutil.rmtree(self._tmp_dir, ignore_errors=True)

    def _new_saved_doc(self, name: str) -> fc.Document:
        # Links between documents require both documents to be saved.
        doc = fc.newDocument(f'{name}_{self._testMethodName}')
        self._doc_names.append(doc.Name)
        doc.saveAs(str(self._tmp_dir / f'{doc.Name}.FCStd'))
        return doc

    def _build(self, external_geometry: bool) -> tuple[fc.Document, fc.DocumentObject]:
        """Build the structure described in the module docstring.

        Return (robot document, dynamic link `real_link_`).
        """
        robot_doc = self._new_saved_doc('Robot')
        geo_doc = self._new_saved_doc('Geometry') if external_geometry else robot_doc

        assembly = geo_doc.addObject('App::Part', 'Assembly')
        assembly.Placement = ASSEMBLY_PLACEMENT
        box = geo_doc.addObject('Part::Box', 'Box')
        box.Length, box.Width, box.Height = 40.0, 20.0, 10.0
        box.Placement = BOX_PLACEMENT
        assembly.addObject(box)
        geo_doc.recompute()

        wrapper = robot_doc.addObject('App::Part', 'Part')
        wrapper.Placement = WRAPPER_PART_PLACEMENT
        link = robot_doc.addObject('App::Link', 'Link')
        link.LinkedObject = assembly
        link.Placement = LINK_IN_WRAPPER_PLACEMENT
        wrapper.addObject(link)

        real = robot_doc.addObject('App::Link', 'real_link_')
        real.LinkedObject = wrapper
        robot_doc.recompute()
        return robot_doc, real

    def _make_lcs(self, real: fc.DocumentObject, sub_element: str = SUB_ELEMENT):
        selection = FakeSelectionObject(real, sub_element)
        return wb_utils.make_lcs_at_link_body(
            selection,
            delete_created_objects=False,
            lcs_concentric_reversed=False,
            deactivate_after_map_mode=True,
        )

    def _expected_lcs_base(self, real: fc.DocumentObject, sub_element: str = SUB_ELEMENT) -> fc.Vector:
        sub_shape = real.getSubObject(sub_element, transform=False)
        if sub_shape.ShapeType == 'Vertex':
            return sub_shape.Point
        return sub_shape.CenterOfMass

    def _assert_lcs_ok(self, real, lcs, body_lcs_wrapper, sub_element: str = SUB_ELEMENT) -> None:
        wrapper = real.getLinkedObject(True)
        self.assertIs(body_lcs_wrapper.getParentGeoFeatureGroup(), wrapper,
                      'temporary Body must be inside the wrapper App::Part')
        self.assertEqual(
            list(lcs.AttachmentSupport), [],
            'LCS must not keep references to objects inside the Real geometry',
        )
        expected = self._expected_lcs_base(real, sub_element)
        actual = lcs.Placement.Base
        self.assertTrue(
            actual.isEqual(expected, 1e-6),
            f'LCS position in wrapper Part frame: actual={actual} expected={expected}',
        )

    def test_real_geometry_in_external_document(self):
        """Reproduces the reported error (geometry in another document)."""
        _, real = self._build(external_geometry=True)
        try:
            lcs, body_lcs_wrapper, _, _ = self._make_lcs(real)
        except ValueError as e:
            self.fail(f'make_lcs_at_link_body() raised {type(e).__name__}: {e}')
        self._assert_lcs_ok(real, lcs, body_lcs_wrapper)

    def test_real_geometry_in_same_document(self):
        """Same structure, geometry in the robot document.

        Used to keep a reference to `Box` (inside the Real geometry) and to
        lose the placement of the wrapper's App::Link.
        """
        _, real = self._build(external_geometry=False)
        lcs, body_lcs_wrapper, _, _ = self._make_lcs(real)
        self._assert_lcs_ok(real, lcs, body_lcs_wrapper)

    def test_object_with_same_name_in_robot_document(self):
        """An unrelated `Box` in the robot document must not be used."""
        robot_doc, real = self._build(external_geometry=True)
        unrelated = robot_doc.addObject('Part::Box', 'Box')
        unrelated.Placement = fc.Placement(fc.Vector(-1000.0, -1000.0, -1000.0), fc.Rotation())
        robot_doc.recompute()
        lcs, body_lcs_wrapper, _, _ = self._make_lcs(real)
        self._assert_lcs_ok(real, lcs, body_lcs_wrapper)

    def test_vertex(self):
        """A vertex gives an LCS at the vertex (MapMode Translate)."""
        _, real = self._build(external_geometry=True)
        lcs, body_lcs_wrapper, _, _ = self._make_lcs(real, 'Link.Box.Vertex8')
        self._assert_lcs_ok(real, lcs, body_lcs_wrapper, 'Link.Box.Vertex8')

    def test_temporary_objects_are_removed(self):
        """Only the Body (with the LCS) is added to the wrapper Part."""
        robot_doc, real = self._build(external_geometry=True)
        wrapper = real.getLinkedObject(True)
        group_before = list(wrapper.Group)
        lcs, body_lcs_wrapper, _, _ = self._make_lcs(real)
        self.assertEqual(list(wrapper.Group), group_before + [body_lcs_wrapper])
        self.assertIn(lcs, body_lcs_wrapper.Group)
        self.assertEqual(
            [o.Name for o in robot_doc.Objects if o.TypeId == 'Part::Feature'], [],
            'temporary copy of the sub-element must be removed',
        )

    def test_delete_created_objects(self):
        robot_doc, real = self._build(external_geometry=True)
        names_before = {o.Name for o in robot_doc.Objects}
        selection = FakeSelectionObject(real, SUB_ELEMENT)
        _, _, placement, _ = wb_utils.make_lcs_at_link_body(selection, delete_created_objects=True)
        self.assertEqual({o.Name for o in robot_doc.Objects}, names_before)
        self.assertTrue(placement.Base.isEqual(self._expected_lcs_base(real), 1e-6))


if __name__ == '__main__':
    unittest.main()
