"""Tests of `freecad_utils.parse_freecad_path()` object resolution.

The path is a selection sub-element name, e.g. `Link.Box.Face6`, relative to
an object whose first path element (`base_name`) is in `doc`. The objects
after an `App::Link` may live in another (external) document, where names
can collide with names of unrelated objects in `doc`.

The old implementation looked up the path elements by name with
`doc.getObject()` and returned the deepest name found in `doc`, which gave
an unrelated object with the same name, or `None` for objects in an external
document. The path must be followed from the base object through groups,
links and documents.

Structure::

    Robot document                          Geometry document (external file)
    --------------                          ---------------------------------
    Part (App::Part)
      Link (App::Link)  ------------------>  Assembly (App::Part)
    Box (Part::Box, unrelated, same Name)      Box (Part::Box)
                                               Body (PartDesign::Body)
                                                 LCS (PartDesign::CoordinateSystem)
"""
from __future__ import annotations

import shutil
import tempfile
import unittest
from pathlib import Path

import FreeCAD as fc

from ..freecad_utils import parse_freecad_path


class ParseFreecadPathTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        cls._tmp_dir = Path(tempfile.mkdtemp(prefix='robotcad_path_test_'))
        # Links between documents require both documents to be saved.
        cls.robot_doc = fc.newDocument('RobotParsePathTest')
        cls.robot_doc.saveAs(str(cls._tmp_dir / f'{cls.robot_doc.Name}.FCStd'))
        cls.geo_doc = fc.newDocument('GeometryParsePathTest')
        cls.geo_doc.saveAs(str(cls._tmp_dir / f'{cls.geo_doc.Name}.FCStd'))

        assembly = cls.geo_doc.addObject('App::Part', 'Assembly')
        cls.box = cls.geo_doc.addObject('Part::Box', 'Box')
        assembly.addObject(cls.box)
        body = cls.geo_doc.addObject('PartDesign::Body', 'Body')
        assembly.addObject(body)
        cls.lcs = cls.geo_doc.addObject('PartDesign::CoordinateSystem', 'LCS')
        body.addObject(cls.lcs)
        cls.geo_doc.recompute()

        part = cls.robot_doc.addObject('App::Part', 'Part')
        cls.link = cls.robot_doc.addObject('App::Link', 'Link')
        cls.link.LinkedObject = assembly
        part.addObject(cls.link)
        cls.unrelated_box = cls.robot_doc.addObject('Part::Box', 'Box')
        cls.robot_doc.recompute()

    @classmethod
    def tearDownClass(cls) -> None:
        for doc in (cls.robot_doc, cls.geo_doc):
            try:
                fc.closeDocument(doc.Name)
            except Exception:
                pass
        shutil.rmtree(cls._tmp_dir, ignore_errors=True)

    def test_preconditions(self):
        self.assertEqual(self.box.Name, 'Box')
        self.assertEqual(self.unrelated_box.Name, 'Box')
        self.assertIsNot(self.box, self.unrelated_box)

    def test_face_of_object_in_external_document(self):
        result = parse_freecad_path('Link.Box.Face6', self.robot_doc)
        self.assertIs(result['object'], self.box)
        self.assertEqual(result['base_name'], 'Link')
        self.assertEqual(result['sub_path'], ['Box'])
        self.assertTrue(result['is_geometry'])
        self.assertEqual(result['geometry_type'], 'Face')
        self.assertEqual(result['geometry_index'], 6)
        self.assertFalse(result['is_datum'])

    def test_tuple_path(self):
        result = parse_freecad_path(('Link.Box.Face6',), self.robot_doc)
        self.assertIs(result['object'], self.box)

    def test_lcs_in_external_document(self):
        for path in ('Link.Body.LCS', 'Link.Body.LCS.'):
            with self.subTest(path=path):
                result = parse_freecad_path(path, self.robot_doc)
                self.assertIs(result['object'], self.lcs)
                self.assertTrue(result['is_datum'])
                self.assertEqual(result['datum_type'], 'LCS')

    def test_lcs_feature_in_external_document(self):
        result = parse_freecad_path('Link.Body.LCS.X', self.robot_doc)
        self.assertIs(result['object'], self.lcs)
        self.assertEqual(result['datum_type'], 'LCS')
        self.assertEqual(result['datum_feature'], 'X')

    def test_mapped_element_name(self):
        result = parse_freecad_path('Link.Box.;#7:1;:G0;XTR;:Hd5,F.Face6', self.robot_doc)
        self.assertIs(result['object'], self.box)
        self.assertEqual(result['geometry_type'], 'Face')
        self.assertEqual(result['geometry_index'], 6)

    def test_object_in_same_document(self):
        result = parse_freecad_path('Part.Link', self.robot_doc)
        self.assertIs(result['object'], self.link)

    def test_base_object_only(self):
        result = parse_freecad_path('Box', self.robot_doc)
        self.assertIs(result['object'], self.unrelated_box)

    def test_broken_path_gives_none(self):
        # Must not fall back to an object found by name (here `Box` exists in
        # both documents).
        result = parse_freecad_path('Link.Nope.Box.Face1', self.robot_doc)
        self.assertIsNone(result['object'])

    def test_unknown_base_gives_none(self):
        result = parse_freecad_path('Nope.Box.Face1', self.robot_doc)
        self.assertIsNone(result['object'])

    def test_without_document(self):
        result = parse_freecad_path('Link.Box.Face6', None)
        self.assertEqual(result['base_name'], 'Link')
        self.assertIsNone(result['object'])


if __name__ == '__main__':
    unittest.main()
