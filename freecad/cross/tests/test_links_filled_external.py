"""`link_proxy.make_robot_links_filled()` with objects of an external document.

Command "NewLinksFilled" ("New links with filled Real, Visual by selected
objects") may be used with objects of another (external) document than the
active one. Like the import of an Assembly, the element wrapper (created in
the active document) links directly to the external object::

    Robot document (active)                      Geometry document (external)
    -----------------------                      ----------------------------
    robot_parts (group, hidden)
      Box (App::Part, wrapper)
        Box (App::Link)  ------------------->  Box (Part::Box)
    l_Box (Cross::Link, Real = Visual = wrapper)

As for an object of the active document, the existing wrapper of the
object is reused. The external object is left untouched: not hidden (its document is not
modified) and not stored in `robot_parts_origins`, which only stores the
parts designed in the active document.

`make_robot_link_filled()` creates a `Cross::Link`, which needs the GUI
(view provider), so it is replaced in these tests by a stand-in that creates
the element wrapper exactly like it does
(`_find_or_create_link_element_wrapper()`), and returns a fake link.
"""
from __future__ import annotations

import shutil
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import FreeCAD as fc

from .. import link_proxy

BOX_PLACEMENT = fc.Placement(fc.Vector(1.0, 2.0, 3.0), fc.Rotation(fc.Vector(0, 0, 1), 30.0))
ASSEMBLY_PLACEMENT = fc.Placement(fc.Vector(10.0, 20.0, 30.0), fc.Rotation())


class _FakeCrossLink:
    def __init__(self, wrapper):
        self.Real = wrapper


class MakeRobotLinksFilledExternalTests(unittest.TestCase):

    def setUp(self) -> None:
        self._tmp_dir = Path(tempfile.mkdtemp(prefix='robotcad_links_filled_test_'))
        self._doc_names: list[str] = []
        # Links between documents require both documents to be saved.
        self.geo_doc = self._new_saved_doc('Geometry')
        assembly = self.geo_doc.addObject('App::Part', 'Assembly')
        assembly.Placement = ASSEMBLY_PLACEMENT
        self.box = self.geo_doc.addObject('Part::Box', 'Box')
        self.box.Length, self.box.Width, self.box.Height = 40.0, 20.0, 10.0
        self.box.Placement = BOX_PLACEMENT
        assembly.addObject(self.box)
        self.geo_doc.recompute()
        # Created last: the active document (where the robot is built).
        self.robot_doc = self._new_saved_doc('Robot')
        fc.setActiveDocument(self.robot_doc.Name)

        self.created_wrappers = []
        patcher_make = mock.patch.object(link_proxy, 'make_robot_link_filled', side_effect=self._fake_make_robot_link_filled)
        patcher_is_link = mock.patch.object(link_proxy, 'is_link', side_effect=lambda o: isinstance(o, _FakeCrossLink))
        self.make_robot_link_filled = patcher_make.start()
        patcher_is_link.start()
        self.addCleanup(patcher_make.stop)
        self.addCleanup(patcher_is_link.stop)

    def tearDown(self) -> None:
        for name in reversed(self._doc_names):
            try:
                fc.closeDocument(name)
            except Exception:
                pass
        shutil.rmtree(self._tmp_dir, ignore_errors=True)

    def _new_saved_doc(self, name: str) -> fc.Document:
        doc = fc.newDocument(f'{name}_{self._testMethodName}')
        self._doc_names.append(doc.Name)
        doc.saveAs(str(self._tmp_dir / f'{doc.Name}.FCStd'))
        return doc

    def _fake_make_robot_link_filled(self, obj, create_parts_group=False, assembly_reference=''):
        doc = fc.ActiveDocument
        part, _created = link_proxy._find_or_create_link_element_wrapper(obj, doc)
        if not part:
            return False
        if create_parts_group:
            link_proxy._store_robot_link_element_part(part, doc)
        self.created_wrappers.append(part)
        return _FakeCrossLink(part)

    def _origins(self) -> list:
        container = self.robot_doc.getObject('robot_parts_origins')
        return list(container.Group) if container else []

    def test_wrapper_links_directly_to_external_object(self):
        links = link_proxy.make_robot_links_filled([self.box])
        self.assertEqual(len(links), 1)
        self.assertIs(self.make_robot_link_filled.call_args.args[0], self.box)
        wrapper = links[0].Real
        self.assertIs(wrapper.Document, self.robot_doc)
        (inner_link,) = wrapper.Group
        self.assertIs(inner_link.LinkedObject, self.box)

    def test_external_object_is_left_untouched(self):
        link_proxy.make_robot_links_filled([self.box])
        self.assertTrue(self.box.Visibility)
        self.assertEqual(self._origins(), [])
        self.assertIsNone(self.geo_doc.getObject('robot_parts_origins'))
        self.assertEqual(
            [o.TypeId for o in self.robot_doc.Objects if o.TypeId == 'App::Link' and o.LinkedObject is self.box],
            ['App::Link'],
            'only the link of the wrapper must link to the external object',
        )

    def test_real_geometry_of_external_object(self):
        """The wrapper shows the object at its local origin, like for a local object."""
        links = link_proxy.make_robot_links_filled([self.box])
        wrapper = links[0].Real
        self.robot_doc.recompute()
        (inner_link,) = wrapper.Group
        face = wrapper.getSubObject(f'{inner_link.Name}.Face6', transform=False)
        self.assertTrue(face.CenterOfMass.isEqual(fc.Vector(20.0, 10.0, 10.0), 1e-6), f'{face.CenterOfMass}')

    def test_wrapper_of_external_object_is_reused(self):
        """Same behavior as for an object of the active document."""
        first = link_proxy.make_robot_links_filled([self.box])
        second = link_proxy.make_robot_links_filled([self.box])
        self.assertIs(first[0].Real, second[0].Real)
        self.assertEqual(len([o for o in self.robot_doc.Objects if o.TypeId == 'App::Part']), 1)

    def test_wrapper_of_local_object_is_reused(self):
        local_box = self.robot_doc.addObject('Part::Box', 'LocalBox')
        self.robot_doc.recompute()
        first = link_proxy.make_robot_links_filled([local_box])
        second = link_proxy.make_robot_links_filled([local_box])
        self.assertIs(first[0].Real, second[0].Real)

    def test_local_object_is_stored_as_origin(self):
        local_box = self.robot_doc.addObject('Part::Box', 'LocalBox')
        self.robot_doc.recompute()
        link_proxy.make_robot_links_filled([local_box])
        self.assertIs(self.make_robot_link_filled.call_args.args[0], local_box)
        self.assertEqual(self._origins(), [local_box])
        self.assertFalse(local_box.Visibility)

    def test_nothing_is_created_on_failure(self):
        names_before = {o.Name for o in self.robot_doc.Objects}
        self.make_robot_link_filled.side_effect = lambda *args, **kwargs: False
        links = link_proxy.make_robot_links_filled([self.box])
        self.assertEqual(links, [])
        self.assertEqual({o.Name for o in self.robot_doc.Objects}, names_before)
        self.assertTrue(self.box.Visibility)

    def test_get_selected_objects_of_all_documents(self):
        if not fc.GuiUp:
            self.skipTest('needs the FreeCAD GUI (selection)')
        import FreeCADGui as fcgui
        local_box = self.robot_doc.addObject('Part::Box', 'LocalBox')
        self.robot_doc.recompute()
        fcgui.Selection.clearSelection()
        try:
            fcgui.Selection.addSelection(self.geo_doc.Name, self.box.Name)
            fcgui.Selection.addSelection(self.robot_doc.Name, local_box.Name)
            selected = link_proxy.get_selected_objects_of_all_documents()
        finally:
            fcgui.Selection.clearSelection()
        self.assertEqual(len(selected), 2)
        self.assertTrue(any(o is self.box for o in selected))
        self.assertTrue(any(o is local_box for o in selected))


if __name__ == '__main__':
    unittest.main()
