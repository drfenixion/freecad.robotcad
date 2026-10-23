"""Smoke test for the `output_level` argument.

This does not try to capture and assert on `FreeCAD.Console` output
(the observer API for that varies across FreeCAD versions and is out of
scope here); it only checks that both output levels run to completion
without raising and produce the same numeric result, i.e. that
`output_level` only affects what is printed, never what is computed.
"""
from __future__ import annotations

from .. import inertia_calculator as ic
from . import materials, shapes
from .base import InertiaCalculatorTestCase

DENSITY_KG_PER_M3 = 7850.0
LENGTH, WIDTH, HEIGHT = 100.0, 40.0, 25.0


class OutputLevelTests(InertiaCalculatorTestCase):

    def test_info_and_warn_levels_produce_the_same_result(self):
        materials.require_materials_module()
        box = shapes.make_part_box(self.doc, 'Box', LENGTH, WIDTH, HEIGHT)
        materials.assign_material(box, materials.make_material('Steel', DENSITY_KG_PER_M3))

        result_info = ic.calculate(box, output_level=ic.LOG_LEVEL_INFO)
        result_warn = ic.calculate(box, output_level=ic.LOG_LEVEL_WARN)
        self.assertInertia6Equal(result_info, result_warn, atol=1e-12, rtol=0.0)

    def test_nested_tree_with_each_output_level_does_not_raise(self):
        materials.require_materials_module()
        box1 = shapes.make_part_box(self.doc, 'Box1', LENGTH, WIDTH, HEIGHT)
        box2 = shapes.make_part_box(self.doc, 'Box2', LENGTH, WIDTH, HEIGHT)
        for box in (box1, box2):
            materials.assign_material(box, materials.make_material('Steel', DENSITY_KG_PER_M3))
        container = shapes.make_part_container(self.doc, 'Container', [box1, box2])

        for level in (ic.LOG_LEVEL_INFO, ic.LOG_LEVEL_WARN):
            with self.subTest(output_level=level):
                ic.calculate(container, output_level=level)
